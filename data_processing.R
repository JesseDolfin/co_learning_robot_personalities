# Load necessary library
library(dplyr)

# Set the working directory to the script's location
setwd(dirname(rstudioapi::getActiveDocumentContext()$path))  # Requires RStudio

# Define the data collection folder path
data_collection_dir <- "/home/jesse/thesis/src/co_learning_robot_personalities/data_collection"

# Debug: Check the folder path
print(data_collection_dir)
print(list.files(data_collection_dir))  # Lists participant folders

# Function to analyze a single log file
analyze_logs <- function(log_file) {
  # Read the CSV file
  results <- read.csv(log_file)
  
  # Calculate total episodes and successful episodes
  total_episodes <- nrow(results)
  successful_episodes <- sum(results$task_status == 1, na.rm = TRUE) # Success is when task_status == 1
  
  # Calculate Mean Performance Rate (MPR)
  MPR <- (successful_episodes / total_episodes) * 100
  
  # Calculate Cumulative Reward
  cumulative_reward <- sum(results$total_reward, na.rm = TRUE)
  
  # Return Metrics as a Named List
  list(
    File = basename(log_file),
    Mean_Performance_Rate = MPR,
    Cumulative_Reward = cumulative_reward
  )
}

# Function to analyze all personality folders for a participant
analyze_participant <- function(participant_dir) {
  # Get a list of all personality folders within the participant directory
  personality_folders <- list.dirs(participant_dir, recursive = FALSE)
  
  # Prepare a data frame to hold results
  participant_results <- data.frame()
  
  # Iterate over personality folders
  for (folder in personality_folders) {
    # Look for the episode_logs.csv file in the personality folder
    log_file <- file.path(folder, "logs", "episode_logs.csv")
    
    if (file.exists(log_file)) {
      # Analyze the log file
      metrics <- analyze_logs(log_file)
      
      # Add participant and personality info to the results
      metrics <- as.data.frame(metrics)
      metrics$Participant <- basename(participant_dir)
      metrics$Personality <- basename(folder)
      
      # Append to the results data frame
      participant_results <- rbind(participant_results, metrics)
    }
  }
  
  return(participant_results)
}

# Function to analyze all participants in the data_collection folder
analyze_all_participants <- function(data_collection_dir) {
  # Get a list of all participant folders
  participant_folders <- list.dirs(data_collection_dir, recursive = FALSE)
  
  # Prepare a data frame to hold all results
  all_results <- data.frame()
  
  # Iterate over participant folders
  for (participant in participant_folders) {
    participant_results <- analyze_participant(participant)
    all_results <- rbind(all_results, participant_results)
  }
  
  return(all_results)
}


# Main script
# Analyze all participants
final_results <- analyze_all_participants(data_collection_dir)

# Print the final results
print(final_results)

# Save the final results to a CSV file in the data_collection directory
write.csv(final_results, 
          file = file.path(data_collection_dir, "summary_metrics_all_participants.csv"), 
          row.names = FALSE)

