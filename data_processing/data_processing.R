# Load necessary libraries
library(dplyr)

# Set the working directory to the script's location
setwd(dirname(rstudioapi::getActiveDocumentContext()$path))  # Requires RStudio

# Define the data collection folder path
data_collection_dir <- "/home/jesse/thesis/src/co_learning_robot_personalities/data_collection"

# Debug: Check the folder path
print(data_collection_dir)
print(list.files(data_collection_dir))  # Lists participant folders

# Function to calculate fluency score from form responses
calculate_fluency <- function(form_file) {
  # Read the form responses
  form_data <- read.csv(form_file)
  
  # Select all columns for fluency based on the provided form
  fluency_columns <- c(
    "The.team.worked.fluently.together.",
    "The.robot.adapted.to.my.input.as.the.task.progressed.",
    "The.robot.contributed.to.the.team.s.success",
    "I.felt.like.an.equal.partner.in.the.team.",
    "I.had.to.guide.the.robot.more.than.expected.",
    "The.robot.made.independent.decisions.when.appropriate.",
    "I.trusted.the.robot.to.act.according.to.our.shared.goals.",
    "I.had.to.constantly.monitor.the.robot.s.actions.to.ensure.task.success.",
    "The.robot.made.decisions.that.aligned.with.my.expectations.for.timing.and.handover.actions.",
    "The.robot.was.committed.to.the.task.",
    "The.robot.made.independent.decisions.when.appropriate.to.support.the.task.",
    "The.robot.improved.over.time.",
    "I.adjusted.my.actions.based.on.the.robot.s.behavior.",
    "The.robot.demonstrated.an.understanding.of.the.shared.task.goals.",
    "The.robot.and.I.shared.a.mutual.understanding.of.the.task.requirements."
  )
  
  if (!all(fluency_columns %in% colnames(form_data))) {
    missing <- setdiff(fluency_columns, colnames(form_data))
    stop(paste("The following fluency-related columns are missing in the form data:", paste(missing, collapse = ", ")))
  }
  
  # Compute the composite fluency score as the mean of relevant responses
  fluency_score <- form_data %>%
    select(all_of(fluency_columns)) %>%
    rowMeans(na.rm = TRUE)
  
  return(fluency_score)
}

# Function to calculate axis scores
calculate_axis_scores <- function(form_file) {
  # Read the form responses
  form_data <- read.csv(form_file)
  
  # Define the columns for the two axes
  axis_columns <- list(
    leader_follower = c(
      "Did.the.robot.appear.to.take.initiative.during.the.interaction.",
      "Did.the.robot.seem.to.adapt.to.your.actions.or.follow.your.lead.",  # (R)
      "Would.you.describe.the.robot.as.assertive.in.its.actions.",
      "Did.you.feel.like.you.had.to.adjust.to.the.robot.s.decisions."  # (R)
    ),
    patient_impatient = c(
      "Did.the.robot.wait.for.you.before.acting.",
      "Did.the.robot.s.behavior.feel.rushed.",  # (R)
      "Did.the.robot.give.you.enough.time.to.complete.your.part.of.the.task.",
      "Did.the.robot.seem.willing.to.accommodate.your.pace."
    )
  )
  
  # Define reversed questions
  reversed_questions <- c(
    "Did.the.robot.seem.to.adapt.to.your.actions.or.follow.your.lead.",
    "Did.you.feel.like.you.had.to.adjust.to.the.robot.s.decisions.",
    "Did.the.robot.s.behavior.feel.rushed."
  )
  
  # Check if all required columns are present
  if (!all(unlist(axis_columns) %in% colnames(form_data))) {
    missing <- setdiff(unlist(axis_columns), colnames(form_data))
    stop(paste("The following axis-related columns are missing in the form data:", paste(missing, collapse = ", ")))
  }
  
  # Function to map responses according to the scoring rules
  map_score <- function(score, reverse = FALSE) {
    mapped <- score - 4  # Shift the score by -4 to center it around 0
    if (reverse) mapped <- -mapped
    return(mapped)
  }
  
  # Helper function to calculate axis scores
  process_scores <- function(columns) {
    is_reversed <- columns %in% reversed_questions
    scores <- mapply(function(column, reverse) {
      map_score(form_data[[column]], reverse)
    }, columns, is_reversed, SIMPLIFY = FALSE)
    rowMeans(do.call(cbind, scores), na.rm = TRUE)
  }
  
  # Calculate scores for both axes
  leader_follower_scores <- process_scores(axis_columns$leader_follower)
  patient_impatient_scores <- process_scores(axis_columns$patient_impatient)
  
  return(list(
    Leader_Follower = leader_follower_scores,
    Patient_Impatient = patient_impatient_scores
  ))
}

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
    
    # Look for the form_responses.csv file in the personality folder
    form_file <- file.path(folder, "form_responses.csv")
    
    if (file.exists(log_file)) {
      # Analyze the log file
      metrics <- analyze_logs(log_file)
      
      # Add participant and personality info to the results
      metrics <- as.data.frame(metrics)
      metrics$Participant <- basename(participant_dir)
      metrics$Personality <- basename(folder)
      
      # If the form responses exist, add the fluency score and axis scores
      if (file.exists(form_file)) {
        metrics$Fluency_Score <- calculate_fluency(form_file)
        axis_scores <- calculate_axis_scores(form_file)
        metrics$Patient_Impatient_Score <- axis_scores$Patient_Impatient
        metrics$Leader_Follower_Score <- axis_scores$Leader_Follower
      } else {
        metrics$Fluency_Score <- NA
        metrics$Patient_Impatient_Score <- NA
        metrics$Leader_Follower_Score <- NA
      }
      
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

