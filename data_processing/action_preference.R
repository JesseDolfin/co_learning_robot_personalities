# Load necessary libraries
library(dplyr)
library(stringr)

data_collection_dir <- "/home/jesse/thesis/src/co_learning_robot_personalities/data_collection"

# Set the threshold 'a' for streak length
a <- 3

# -------------------------------------------------------------------
# Helper function: Determine the phase based on the state number
# -------------------------------------------------------------------
get_phase <- function(state) {
  # Mapping state ranges to phases:
  # state == 0 -> phase 0
  # 1:4 -> phase 1
  # 5:10 -> phase 2
  # 11:16 -> phase 3
  if (state == 0) {
    return(0)
  } else if (state %in% 1:4) {
    return(1)
  } else if (state %in% 5:10) {
    return(2)
  } else if (state %in% 11:16) {
    return(3)
  } else {
    warning("No valid phase found for state: ", state)
    return(NA_integer_)
  }
}

# -------------------------------------------------------------------
# Given a Q-table, pick for each phase the action corresponding to the single highest Q-value.
# -------------------------------------------------------------------
compute_actions_per_phase <- function(q_table) {
  states <- 0:(nrow(q_table)-1)
  phases <- sapply(states, get_phase)
  
  actions_per_phase <- integer(4) # store chosen action for phases 0 to 3
  
  for (p in 0:3) {
    phase_states <- which(phases == p)
    if (length(phase_states) == 0) {
      # No states for this phase, store NA
      actions_per_phase[p+1] <- NA
      next
    }
    
    # Extract Q-values for these phase states
    q_sub <- q_table[phase_states, , drop = FALSE]
    
    # Find the global maximum Q-value in this submatrix
    max_val <- max(q_sub)
    # Get the indices (row and column) of this max value
    max_pos <- which(q_sub == max_val, arr.ind = TRUE)
    # chosen_action is the column (action) index of the max Q-value
    chosen_action <- max_pos[1, "col"]  # If multiple maxima, take the first
    actions_per_phase[p+1] <- chosen_action
  }
  
  return(actions_per_phase)
}

# -------------------------------------------------------------------
# Analyze all Q_tables in a given Q_table directory (for a single personality).
# Returns a matrix of dimensions (#episodes x 4), where rows = episodes, cols = phases.
# -------------------------------------------------------------------
analyze_personality <- function(q_table_path) {
  q_table_files <- list.files(q_table_path, pattern="Q_table_\\d+\\.csv", full.names=TRUE)
  
  if (length(q_table_files) == 0) {
    warning("No Q-tables found in: ", q_table_path)
    return(matrix(NA, nrow=0, ncol=4))
  }
  
  # Extract episode from filename and sort by episode number to ensure proper ordering
  episode_numbers <- as.integer(str_extract(basename(q_table_files), "(?<=Q_table_)\\d+"))
  order_idx <- order(episode_numbers)
  q_table_files <- q_table_files[order_idx]
  
  all_phase_actions <- lapply(q_table_files, function(f) {
    q_table <- tryCatch({
      read.csv(f, header=FALSE)
    }, error = function(e) {
      warning("Error reading Q-table: ", f, " - ", e$message)
      return(NULL)
    })
    
    if (is.null(q_table)) {
      return(rep(NA, 4))
    }
    
    # Compute chosen action per phase
    compute_actions_per_phase(q_table)
  })
  
  phase_action_matrix <- do.call(rbind, all_phase_actions)
  return(phase_action_matrix)
}

# -------------------------------------------------------------------
# Original stability count function (consecutive identical pairs)
# -------------------------------------------------------------------
count_stability <- function(actions_per_phase_matrix) {
  # actions_per_phase_matrix: rows = episodes, cols = phases
  if (nrow(actions_per_phase_matrix) <= 1) {
    return(rep(0, ncol(actions_per_phase_matrix)))
  }
  
  apply(actions_per_phase_matrix, 2, function(actions) {
    sum(actions[-1] == actions[-length(actions)])
  })
}

# -------------------------------------------------------------------
# New function to count streaks and longest streak based on a threshold 'a'
# -------------------------------------------------------------------
count_streaks <- function(actions, a) {
  if (length(actions) == 0 || all(is.na(actions))) {
    return(list(streak_count = 0, longest_streak = 0))
  }
  
  actions <- na.omit(actions) # remove NA if any
  if (length(actions) == 0) {
    return(list(streak_count = 0, longest_streak = 0))
  }
  
  streak_count <- 0
  longest_streak <- 1
  
  current_action <- actions[1]
  current_length <- 1
  
  for (i in 2:length(actions)) {
    if (actions[i] == current_action) {
      # Continue the streak
      current_length <- current_length + 1
    } else {
      # Streak ended
      if (current_length >= a) {
        streak_count <- streak_count + 1
      }
      if (current_length > longest_streak) {
        longest_streak <- current_length
      }
      # Reset for the new action
      current_action <- actions[i]
      current_length <- 1
    }
  }
  
  # Check the final streak
  if (current_length >= a) {
    streak_count <- streak_count + 1
  }
  if (current_length > longest_streak) {
    longest_streak <- current_length
  }
  
  return(list(streak_count = streak_count, longest_streak = longest_streak))
}

# -------------------------------------------------------------------
# Function to apply streak counting per phase
# -------------------------------------------------------------------
compute_streak_metrics <- function(actions_per_phase_matrix, a) {
  # actions_per_phase_matrix: rows = episodes, cols = phases
  # Returns two matrices: streak_count_per_phase and longest_streak_per_phase
  # each with one row (since it's for a single participant/personality) and 4 columns (one per phase)
  
  if (nrow(actions_per_phase_matrix) == 0) {
    # No data
    return(list(
      StreakCount = c(NA, NA, NA, NA),
      LongestStreak = c(NA, NA, NA, NA)
    ))
  }
  
  phases <- 1:ncol(actions_per_phase_matrix)
  
  streak_counts <- numeric(length(phases))
  longest_streaks <- numeric(length(phases))
  
  for (p in phases) {
    phase_actions <- actions_per_phase_matrix[, p]
    res <- count_streaks(phase_actions, a)
    streak_counts[p] <- res$streak_count
    longest_streaks[p] <- res$longest_streak
  }
  
  return(list(StreakCount = streak_counts, LongestStreak = longest_streaks))
}

# -------------------------------------------------------------------
# Main code: 
# 1) Iterate over all participants and personalities, 
# 2) Compute stability counts,
# 3) Compute streak metrics,
# 4) Aggregate results by personality type,
# 5) Write out personality_average_stability.csv
# -------------------------------------------------------------------

participants <- list.dirs(data_collection_dir, recursive=FALSE, full.names=TRUE)
participants <- participants[grepl("participant_", basename(participants))]

# Initialize an empty data frame to store all results
all_results <- data.frame(
  Participant = character(),
  Personality = character(),
  Stability_Phase0 = integer(),
  Stability_Phase1 = integer(),
  Stability_Phase2 = integer(),
  Stability_Phase3 = integer(),
  StreakCount_Phase0 = integer(),
  StreakCount_Phase1 = integer(),
  StreakCount_Phase2 = integer(),
  StreakCount_Phase3 = integer(),
  LongestStreak_Phase0 = integer(),
  LongestStreak_Phase1 = integer(),
  LongestStreak_Phase2 = integer(),
  LongestStreak_Phase3 = integer(),
  stringsAsFactors = FALSE
)

for (participant_dir in participants) {
  participant_name <- basename(participant_dir)
  
  # List personality directories for this participant
  personality_dirs <- list.dirs(participant_dir, recursive=FALSE, full.names=TRUE)
  personality_dirs <- personality_dirs[grepl("personality_type_", basename(personality_dirs))]
  
  for (personality_dir in personality_dirs) {
    personality_name <- basename(personality_dir)
    
    # Q_table path
    q_table_path <- file.path(personality_dir, "Q_tables")
    if (!dir.exists(q_table_path)) {
      warning("No Q_tables directory for: ", personality_dir)
      next
    }
    
    # Analyze personality
    phase_action_matrix <- analyze_personality(q_table_path)
    
    if (nrow(phase_action_matrix) == 0) {
      # No data, skip
      next
    }
    
    # Compute old stability metric (consecutive identical pairs)
    final_stability <- count_stability(phase_action_matrix)
    
    # Compute new streak metrics
    streak_res <- compute_streak_metrics(phase_action_matrix, a)
    
    # Add a row to all_results
    new_row <- data.frame(
      Participant = participant_name,
      Personality = personality_name,
      Stability_Phase0 = final_stability[1],
      Stability_Phase1 = final_stability[2],
      Stability_Phase2 = final_stability[3],
      Stability_Phase3 = final_stability[4],
      StreakCount_Phase0 = streak_res$StreakCount[1],
      StreakCount_Phase1 = streak_res$StreakCount[2],
      StreakCount_Phase2 = streak_res$StreakCount[3],
      StreakCount_Phase3 = streak_res$StreakCount[4],
      LongestStreak_Phase0 = streak_res$LongestStreak[1],
      LongestStreak_Phase1 = streak_res$LongestStreak[2],
      LongestStreak_Phase2 = streak_res$LongestStreak[3],
      LongestStreak_Phase3 = streak_res$LongestStreak[4],
      stringsAsFactors = FALSE
    )
    
    all_results <- rbind(all_results, new_row)
  }
}

# Now compute the average stability and streak metrics per personality type
average_stability <- all_results %>%
  group_by(Personality) %>%
  summarize(
    Avg_Stability_Phase0 = mean(Stability_Phase0, na.rm = TRUE),
    Avg_Stability_Phase1 = mean(Stability_Phase1, na.rm = TRUE),
    Avg_Stability_Phase2 = mean(Stability_Phase2, na.rm = TRUE),
    Avg_Stability_Phase3 = mean(Stability_Phase3, na.rm = TRUE),
    Avg_StreakCount_Phase0 = mean(StreakCount_Phase0, na.rm = TRUE),
    Avg_StreakCount_Phase1 = mean(StreakCount_Phase1, na.rm = TRUE),
    Avg_StreakCount_Phase2 = mean(StreakCount_Phase2, na.rm = TRUE),
    Avg_StreakCount_Phase3 = mean(StreakCount_Phase3, na.rm = TRUE),
    Avg_LongestStreak_Phase0 = mean(LongestStreak_Phase0, na.rm = TRUE),
    Avg_LongestStreak_Phase1 = mean(LongestStreak_Phase1, na.rm = TRUE),
    Avg_LongestStreak_Phase2 = mean(LongestStreak_Phase2, na.rm = TRUE),
    Avg_LongestStreak_Phase3 = mean(LongestStreak_Phase3, na.rm = TRUE),
    .groups = "drop"
  ) %>%
  mutate(across(where(is.numeric), round, digits = 3))

# Write out the final aggregated results
output_file <- file.path(data_collection_dir, "personality_average_stability.csv")
write.csv(average_stability, file = output_file, row.names = FALSE)

message("Averaged stability and streak metrics per personality type written to: ", output_file)
