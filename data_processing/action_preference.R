################################################################################
# Load necessary libraries
################################################################################
library(dplyr)
library(stringr)

################################################################################
# Global Configuration
################################################################################
data_collection_dir <- "/home/jesse/thesis/src/co_learning_robot_personalities/data_collection"

# Threshold 'a' for streak length
a <- 3

################################################################################
# Helper Functions
################################################################################

# -------------------------------------------------------------------
# Determine the phase based on the state number
# -------------------------------------------------------------------
get_phase <- function(state) {
  # Match the Python phase definition logic
  if (state == 0) {
    return(0)
  } else if (state %in% c(1, 2, 3, 4)) {
    return(1)
  } else if (state %in% c(5, 6, 7, 8, 9, 10)) {
    return(2)
  } else if (state %in% c(12, 13, 15, 16, 17)) {
    return(3)
  } else if (state %in% c(11, 14)) {
    return(4)
  } else {
    warning("No valid phase found for state: ", state)
    return(NA_integer_)
  }
}


# -------------------------------------------------------------------
# Given a Q-table, pick for each phase the action corresponding
# to the single highest Q-value.
# -------------------------------------------------------------------
compute_actions_per_phase <- function(q_table) {
  states <- 0:(nrow(q_table)-1)
  phases <- sapply(states, get_phase)
  
  actions_per_phase <- integer(4)  # store chosen action for phases 0 to 3
  
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
# Count how many consecutive identical pairs of actions
# in consecutive episodes (older "stability" metric).
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
# Count streaks of length >= a for each sequence
# (Used in new streak metrics).
# -------------------------------------------------------------------
count_streaks <- function(actions, a) {
  if (length(actions) == 0 || all(is.na(actions))) {
    return(list(streak_count = 0, longest_streak = 0))
  }
  
  actions <- na.omit(actions) # remove NAs if any
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
# Apply streak counting per phase
# -------------------------------------------------------------------
compute_streak_metrics <- function(actions_per_phase_matrix, a) {
  # actions_per_phase_matrix: rows = episodes, cols = phases
  # Returns two vectors (StreakCount, LongestStreak) each length=4
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

################################################################################
# New Metrics for Q-table Analysis (Entropy, Gap, Convergence, Consistency)
################################################################################

# -------------------------------------------------------------------
# Softmax of a numeric vector (for computing entropy).
# -------------------------------------------------------------------
softmax <- function(x, temp=1.0) {
  exp_x <- exp(x / temp)
  exp_x / sum(exp_x)
}

# -------------------------------------------------------------------
# Shannon Entropy of a probability distribution vector.
# -------------------------------------------------------------------
shannon_entropy <- function(prob_vec) {
  prob_vec <- prob_vec[prob_vec > 0]  # avoid log(0)
  -sum(prob_vec * log(prob_vec))
}

# -------------------------------------------------------------------
# 1) Entropy of Q-values:
#    - Convert each state’s Q-values to a softmax distribution
#    - Compute Shannon entropy
#    - Average across states
# -------------------------------------------------------------------
compute_q_entropy <- function(q_table) {
  num_states <- nrow(q_table)
  entropies <- numeric(num_states)
  
  for (s in seq_len(num_states)) {
    p_actions <- softmax(q_table[s, ])
    entropies[s] <- shannon_entropy(p_actions)
  }
  mean(entropies, na.rm = TRUE)
}

# -------------------------------------------------------------------
# 2) Average Q-value Gap:
#    - For each state, find difference between highest and 2nd highest action
#    - Average across states
# -------------------------------------------------------------------
compute_q_gap <- function(q_table) {
  # Ensure q_table is a numeric matrix
  q_table <- as.matrix(q_table)
  mode(q_table) <- "numeric"  # Ensure all entries are numeric
  
  num_states <- nrow(q_table)
  gaps <- numeric(num_states)
  
  for (s in seq_len(num_states)) {
    # Extract row as a numeric vector
    row_vals <- q_table[s, ]
    
    # Handle missing or invalid rows
    if (is.null(row_vals) || all(is.na(row_vals))) {
      gaps[s] <- NA
      next
    }
    
    # Sort and compute the gap
    sorted_vals <- sort(row_vals, decreasing = TRUE)
    if (length(sorted_vals) < 2) {
      gaps[s] <- NA
    } else {
      gaps[s] <- sorted_vals[1] - sorted_vals[2]
    }
  }
  
  # Return mean of gaps
  mean(gaps, na.rm = TRUE)
}


# -------------------------------------------------------------------
# 3) Convergence Rate:
#    - L1 distance between Q-tables from consecutive episodes
# -------------------------------------------------------------------
compute_q_distance <- function(q_table1, q_table2) {
  if (any(dim(q_table1) != dim(q_table2))) {
    warning("Q-tables have different dimensions.")
    return(NA)
  }
  sum(abs(q_table1 - q_table2))  # L1 norm
}

# -------------------------------------------------------------------
# 4) Action Consistency:
#    - For each state, check if best action in table1 == best action in table2
#    - Fraction of states consistent
# -------------------------------------------------------------------
action_consistency_for_two_tables <- function(q_table1, q_table2) {
  if (any(dim(q_table1) != dim(q_table2))) {
    warning("Q-tables have different dimensions for action consistency.")
    return(NA)
  }
  
  num_states <- nrow(q_table1)
  consistent_count <- 0
  
  for (s in seq_len(num_states)) {
    best_action_1 <- which.max(q_table1[s, ])
    best_action_2 <- which.max(q_table2[s, ])
    if (best_action_1 == best_action_2) {
      consistent_count <- consistent_count + 1
    }
  }
  
  consistent_count / num_states
}

################################################################################
# Main Analysis Function
################################################################################

# -------------------------------------------------------------------
# Analyze all Q_tables in a given Q_table directory (for a single personality).
# Returns:
#   1) phase_action_matrix (#episodes x 4), rows = episodes, cols = phases
#   2) metrics_df (data frame of episode-level new metrics:
#      Episode, Entropy, AvgQGap, Convergence, ActionConsistency)
# -------------------------------------------------------------------
analyze_personality <- function(q_table_path) {
  q_table_files <- list.files(q_table_path, pattern="Q_table_\\d+\\.csv", full.names=TRUE)
  
  if (length(q_table_files) == 0) {
    warning("No Q-tables found in: ", q_table_path)
    return(list(
      phase_action_matrix = matrix(NA, nrow=0, ncol=4),
      metrics_df = data.frame()
    ))
  }
  
  # Sort by episode number to ensure proper ordering
  temp_vec <- str_extract(basename(q_table_files), "Q_table_\\d+")
  # Then remove the "Q_table_" part
  temp_vec <- str_remove(temp_vec, "Q_table_")
  # Now convert to integer
  episode_numbers <- as.integer(temp_vec)
  
  order_idx <- order(episode_numbers)
  q_table_files <- q_table_files[order_idx]
  episode_numbers <- episode_numbers[order_idx]
  
  phase_action_list <- list()
  
  
  # Storage for new Q-table-based metrics
  metrics_list <- list()
  
  previous_q_table <- NULL
  
  
  for (i in seq_along(q_table_files)) {
    f <- q_table_files[i]
    episode <- episode_numbers[i]
    
    q_table <- tryCatch({
      read.csv(f, header=FALSE)
    }, error = function(e) {
      warning("Error reading Q-table: ", f, " - ", e$message)
      return(NULL)
    })
    
    if (is.null(q_table)) {
      # If there's an error reading, store NAs
      phase_action_list[[i]] <- rep(NA, 4)
      metrics_list[[i]] <- data.frame(
        Episode = episode,
        Entropy = NA,
        AvgQGap = NA,
        Convergence = NA,
        ActionConsistency = NA
      )
      next
    }
   
    
    # 1) Phase-based best-action analysis
    actions_per_phase <- compute_actions_per_phase(q_table)
    phase_action_list[[i]] <- actions_per_phase

    # 2) Compute new metrics
    entropy_val <- compute_q_entropy(q_table)

    q_gap_val   <- compute_q_gap(q_table)

    if (!is.null(previous_q_table)) {
      convergence_val    <- compute_q_distance(previous_q_table, q_table)
      consistency_val    <- action_consistency_for_two_tables(previous_q_table, q_table)
    } else {
      convergence_val    <- NA
      consistency_val    <- NA
    }
    
    previous_q_table <- q_table

    
    metrics_list[[i]] <- data.frame(
      Episode           = episode,
      Entropy           = entropy_val,
      AvgQGap           = q_gap_val,
      Convergence       = convergence_val,
      ActionConsistency = consistency_val
    )
  }
  
  phase_action_matrix <- do.call(rbind, phase_action_list)
  metrics_df <- do.call(rbind, metrics_list)
  
  return(list(
    phase_action_matrix = phase_action_matrix,
    metrics_df          = metrics_df
  ))
}

################################################################################
# Main Code
################################################################################

participants <- list.dirs(data_collection_dir, recursive=FALSE, full.names=TRUE)
participants <- participants[grepl("participant_", basename(participants))]

# 1) Data Frame to store old metrics (stability, streaks) aggregated by participant/personality
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

# 2) Data Frame to store new Q-table-based metrics (per-episode & then aggregated)
all_metrics <- data.frame(
  Participant = character(),
  Personality = character(),
  Episode = integer(),
  Entropy = numeric(),
  AvgQGap = numeric(),
  Convergence = numeric(),
  ActionConsistency = numeric(),
  stringsAsFactors = FALSE
)

# Loop over participants and personalities
for (participant_dir in participants) {
  participant_name <- basename(participant_dir)
  
  # List personality directories
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

    analysis_res <- analyze_personality(q_table_path)
    phase_action_matrix <- analysis_res$phase_action_matrix
    metrics_df <- analysis_res$metrics_df
    
    if (nrow(phase_action_matrix) == 0) {
      # No data, skip
      next
    }
    
    # -------------------------------
    # 1) Old metrics: stability, streaks
    # -------------------------------
    final_stability <- count_stability(phase_action_matrix)
    streak_res <- compute_streak_metrics(phase_action_matrix, a)
    
    new_row <- data.frame(
      Participant           = participant_name,
      Personality           = personality_name,
      Stability_Phase0      = final_stability[1],
      Stability_Phase1      = final_stability[2],
      Stability_Phase2      = final_stability[3],
      Stability_Phase3      = final_stability[4],
      StreakCount_Phase0    = streak_res$StreakCount[1],
      StreakCount_Phase1    = streak_res$StreakCount[2],
      StreakCount_Phase2    = streak_res$StreakCount[3],
      StreakCount_Phase3    = streak_res$StreakCount[4],
      LongestStreak_Phase0  = streak_res$LongestStreak[1],
      LongestStreak_Phase1  = streak_res$LongestStreak[2],
      LongestStreak_Phase2  = streak_res$LongestStreak[3],
      LongestStreak_Phase3  = streak_res$LongestStreak[4],
      stringsAsFactors = FALSE
    )
    
    all_results <- rbind(all_results, new_row)
    
    # -------------------------------
    # 2) New Q-table-based metrics (per-episode)
    # -------------------------------
    if (nrow(metrics_df) > 0) {
      metrics_df$Participant <- participant_name
      metrics_df$Personality <- personality_name
      metrics_df <- metrics_df[, c("Participant", "Personality", "Episode",
                                   "Entropy", "AvgQGap", "Convergence", "ActionConsistency")]

      all_metrics <- rbind(all_metrics, metrics_df)
      
      # Save raw per-episode metrics
      per_episode_file <- file.path(data_collection_dir, "per_episode_metrics.csv")
      write.csv(all_metrics, per_episode_file, row.names = FALSE)
      message("Per-episode metrics saved to: ", per_episode_file)
    }
  }
}

################################################################################
# Aggregate Results (per-participant, per-personality)
################################################################################

# 1) Average stability & streak metrics (per-participant, per-personality)
average_stability <- all_results %>%
  group_by(Participant, Personality) %>%         # <--- group by participant + personality
  summarize(
    Avg_Stability_Phase0       = mean(Stability_Phase0, na.rm = TRUE),
    Avg_Stability_Phase1       = mean(Stability_Phase1, na.rm = TRUE),
    Avg_Stability_Phase2       = mean(Stability_Phase2, na.rm = TRUE),
    Avg_Stability_Phase3       = mean(Stability_Phase3, na.rm = TRUE),
    Avg_StreakCount_Phase0     = mean(StreakCount_Phase0, na.rm = TRUE),
    Avg_StreakCount_Phase1     = mean(StreakCount_Phase1, na.rm = TRUE),
    Avg_StreakCount_Phase2     = mean(StreakCount_Phase2, na.rm = TRUE),
    Avg_StreakCount_Phase3     = mean(StreakCount_Phase3, na.rm = TRUE),
    Avg_LongestStreak_Phase0   = mean(LongestStreak_Phase0, na.rm = TRUE),
    Avg_LongestStreak_Phase1   = mean(LongestStreak_Phase1, na.rm = TRUE),
    Avg_LongestStreak_Phase2   = mean(LongestStreak_Phase2, na.rm = TRUE),
    Avg_LongestStreak_Phase3   = mean(LongestStreak_Phase3, na.rm = TRUE),
    .groups = "drop"
  ) %>%
  # Convert numeric columns to 3 decimal places
  mutate(across(where(is.numeric), round, digits = 3)) %>%
  # (Optional) remove baseline if you do not want to include it
  filter(Personality != "personality_type_baseline")

# Write out aggregated stability & streak metrics
output_file <- file.path(data_collection_dir, "participant_personality_average_stability.csv")
write.csv(average_stability, file = output_file, row.names = FALSE)
print(average_stability)
message("Averaged stability and streak metrics (per participant, per personality) written to: ", output_file)


# 2) Average new metrics (per-participant, per-personality)
average_metrics <- all_metrics %>%
  group_by(Participant, Personality) %>%         # <--- group by participant + personality
  summarize(
    Avg_Entropy            = mean(as.numeric(Entropy), na.rm = TRUE),
    Avg_QGap               = mean(as.numeric(AvgQGap), na.rm = TRUE),
    Avg_Convergence        = mean(as.numeric(Convergence), na.rm = TRUE),
    Avg_ActionConsistency  = mean(as.numeric(ActionConsistency), na.rm = TRUE),
    .groups = "drop"
  ) %>%
  mutate(across(where(is.numeric), round, digits = 3)) %>%
  filter(Personality != "personality_type_baseline")

# Write out aggregated new Q-table-based metrics
output_file_new <- file.path(data_collection_dir, "q_table_metrics.csv")
write.csv(average_metrics, file = output_file_new, row.names = FALSE)
print(average_metrics)
message("Averaged new Q-table metrics (per participant, per personality) written to: ", output_file_new)

