# Load necessary libraries
library(dplyr)
library(stringr)
library(tibble)

# Set the main data collection directory
data_collection_dir <- "/home/jesse/thesis/src/co_learning_robot_personalities/data_collection"

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
    # If the state is outside expected ranges, warn and return NA
    warning("No valid phase found for state: ", state)
    return(NA_integer_)
  }
}

# -------------------------------------------------------------------
# Compute the dominant (preferred) action per phase from a Q-table
# -------------------------------------------------------------------
calculate_phase_dominant_actions <- function(q_table_file, participant_name, personality_name) {
  message("Processing Q-table: ", q_table_file)
  
  # Try reading the Q-table
  q_table <- tryCatch(
    {
      read.csv(q_table_file, header = FALSE)
    },
    error = function(e) {
      warning("Error reading Q-table (", q_table_file, "): ", e$message)
      return(NULL)
    }
  )
  
  # If Q-table not read, return empty tibble
  if (is.null(q_table)) {
    return(tibble(
      Participant = character(),
      Personality = character(),
      Q_Table_File = character(),
      Phase = integer(),
      Dominant_Action = integer()
    ))
  }
  
  # Compute preferred actions (argmax in each state)
  preferred_actions <- tryCatch(
    {
      apply(q_table, 1, which.max)
    },
    error = function(e) {
      warning("Error computing argmax for Q-table (", q_table_file, "): ", e$message)
      return(NULL)
    }
  )
  
  # If preferred actions couldn't be computed, return empty
  if (is.null(preferred_actions)) {
    return(tibble(
      Participant = character(),
      Personality = character(),
      Q_Table_File = character(),
      Phase = integer(),
      Dominant_Action = integer()
    ))
  }
  
  # States assumed to start at 0
  states <- seq_len(nrow(q_table)) - 1
  phases <- sapply(states, get_phase)
  
  # For each phase (0 to 3), find the dominant action
  result_list <- list()
  
  for (p in 0:3) {
    phase_indices <- which(phases == p)
    if (length(phase_indices) == 0) {
      # No states in this phase, skip
      next
    }
    
    # Actions chosen in this phase
    phase_actions <- preferred_actions[phase_indices]
    action_counts <- table(phase_actions)
    
    # Dominant action: action with highest frequency
    dominant_action <- as.integer(names(which.max(action_counts)))
    
    # Create a row for this phase
    df_row <- tibble(
      Participant = participant_name,
      Personality = personality_name,
      Q_Table_File = basename(q_table_file),
      Phase = p,
      Dominant_Action = dominant_action
    )
    result_list[[length(result_list) + 1]] <- df_row
  }
  
  # Combine all phases for this Q-table
  if (length(result_list) > 0) {
    return(bind_rows(result_list))
  } else {
    # If no phases had states or no data was processed
    return(tibble(
      Participant = participant_name,
      Personality = personality_name,
      Q_Table_File = basename(q_table_file),
      Phase = integer(),
      Dominant_Action = integer()
    ))
  }
}

# -------------------------------------------------------------------
# Analyze all Q-tables for a single personality of a participant
# -------------------------------------------------------------------
analyze_personality_phase_preferences <- function(personality_folder, participant_name) {
  q_table_path <- file.path(personality_folder, "Q_tables")
  
  # List Q_table files following the pattern "Q_table_<number>.csv"
  q_table_files <- list.files(
    path = q_table_path,
    pattern = "Q_table_\\d+\\.csv",
    full.names = TRUE
  )
  
  if (length(q_table_files) == 0) {
    warning("No Q-tables found in: ", personality_folder)
    return(tibble(
      Participant = character(),
      Personality = character(),
      Q_Table_File = character(),
      Phase = integer(),
      Dominant_Action = integer()
    ))
  }
  
  personality_name <- basename(personality_folder)
  
  # Process each Q-table file
  all_q_results <- lapply(
    q_table_files, 
    calculate_phase_dominant_actions, 
    participant_name = participant_name, 
    personality_name = personality_name
  )
  
  personality_results <- bind_rows(all_q_results)
  return(personality_results)
}

# -------------------------------------------------------------------
# Analyze all personalities for a single participant
# -------------------------------------------------------------------
analyze_participant_phase_preferences <- function(participant_dir) {
  personality_folders <- list.dirs(participant_dir, recursive = FALSE)
  participant_name <- basename(participant_dir)
  
  if (length(personality_folders) == 0) {
    warning("No personality folders found for participant: ", participant_name)
    return(tibble(
      Participant = character(),
      Personality = character(),
      Q_Table_File = character(),
      Phase = integer(),
      Dominant_Action = integer()
    ))
  }
  
  all_personality_results <- lapply(
    personality_folders, 
    analyze_personality_phase_preferences, 
    participant_name = participant_name
  )
  
  participant_results <- bind_rows(all_personality_results)
  return(participant_results)
}

# -------------------------------------------------------------------
# Analyze all participants in the data collection folder
# -------------------------------------------------------------------
analyze_all_participant_phase_preferences <- function(data_collection_dir) {
  participant_folders <- list.dirs(data_collection_dir, recursive = FALSE)
  
  if (length(participant_folders) == 0) {
    warning("No participant folders found in data collection directory.")
    return(tibble(
      Participant = character(),
      Personality = character(),
      Q_Table_File = character(),
      Phase = integer(),
      Dominant_Action = integer()
    ))
  }
  
  all_results <- lapply(participant_folders, analyze_participant_phase_preferences)
  final_results <- bind_rows(all_results)
  
  return(final_results)
}

# -------------------------------------------------------------------
# Main Execution
# -------------------------------------------------------------------
final_action_phase_preferences <- analyze_all_participant_phase_preferences(data_collection_dir)

# Extract episode number from Q_Table_File (assuming format Q_table_<number>.csv)
# and add as a column "Episode"
final_action_phase_preferences <- final_action_phase_preferences %>%
  mutate(Episode = as.integer(str_extract(Q_Table_File, "(?<=Q_table_)\\d+")))

# Define the length of episodes needed for convergence
required_run_length <- 6

# -------------------------------------------------------------------
# Count convergence events per (Participant, Personality, Phase)
# A convergence event is a stable run of the same action for >= required_run_length episodes
# -------------------------------------------------------------------
count_convergences <- function(d) {
  # Ensure sorted by Episode
  d <- d %>% arrange(Episode)
  
  convergence_count <- 0
  current_action <- NA
  consecutive_count <- 0
  
  for (i in seq_len(nrow(d))) {
    action <- d$Dominant_Action[i]
    if (is.na(current_action) || action != current_action) {
      # Action changed or first action in series
      current_action <- action
      consecutive_count <- 1
    } else {
      # Same action as previous
      consecutive_count <- consecutive_count + 1
    }
    
    # Check if we hit the required stable run length
    if (consecutive_count == required_run_length) {
      convergence_count <- convergence_count + 1
      # We do not reset after this event; subsequent episodes with the same action don't create
      # a new event until the action changes and reconverges.
    }
  }
  
  return(convergence_count)
}

convergence_results <- final_action_phase_preferences %>%
  group_by(Participant, Personality, Phase) %>%
  summarize(
    Convergence_Count = count_convergences(cur_data()),
    .groups = "drop"
  )

# Aggregate results by personality
personality_summary <- convergence_results %>%
  group_by(Personality) %>%
  summarize(
    Avg_Convergence = mean(Convergence_Count, na.rm = TRUE),
    Median_Convergence = median(Convergence_Count, na.rm = TRUE),
    .groups = "drop"
  )

# Save results to CSV files
output_file_convergence <- file.path(data_collection_dir, "convergence_events_summary.csv")
write.csv(convergence_results, file = output_file_convergence, row.names = FALSE)

output_file_personality <- file.path(data_collection_dir, "personality_convergence_aggregate.csv")
write.csv(personality_summary, file = output_file_personality, row.names = FALSE)

# Debug prints
print(convergence_results)
print(personality_summary)
message("Convergence analysis completed and results written to CSV.")
