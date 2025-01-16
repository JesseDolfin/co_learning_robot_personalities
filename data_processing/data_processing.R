# Load necessary libraries
library(dplyr)
library(purrr)
library(readr)

# Optional: Set working directory to the script's location
# This requires RStudio. If you're running this in a different environment, consider removing it.
if (requireNamespace("rstudioapi", quietly = TRUE)) {
  setwd(dirname(rstudioapi::getActiveDocumentContext()$path))
}

# Define the data collection directory
data_collection_dir <- "/home/jesse/thesis/src/co_learning_robot_personalities/data_collection"

# Debug: Check the folder path
message("Data collection directory: ", data_collection_dir)
message("Participant folders found: ", paste(list.files(data_collection_dir), collapse = ", "))

# ---------------------------
# Helper functions
# ---------------------------

# Compute a fluency score from a given form CSV file.
calculate_fluency <- function(form_file) {
  form_data <- read.csv(form_file)
  
  # List of columns required to compute fluency
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
  
  # Check if all fluency columns exist
  if (!all(fluency_columns %in% colnames(form_data))) {
    missing <- setdiff(fluency_columns, colnames(form_data))
    stop("Missing fluency columns: ", paste(missing, collapse = ", "))
  }
  
  # Calculate row means (if multiple rows exist, we take the average)
  fluency_scores <- form_data %>%
    dplyr::select(dplyr::all_of(fluency_columns)) %>%
    rowMeans(na.rm = TRUE)
  
  # If multiple rows, return the mean over all rows
  return(mean(fluency_scores, na.rm = TRUE))
}

calculate_axis_scores <- function(form_file) {
  form_data <- read.csv(form_file)
  
  axis_columns <- list(
    leader_follower = c(
      "Did.the.robot.appear.to.take.initiative.during.the.interaction.",
      "Did.the.robot.seem.to.adapt.to.your.actions.or.follow.your.lead.",  # (R)
      "Would.you.describe.the.robot.as.assertive.in.its.actions.",
      "Did.you.feel.like.you.had.to.adjust.to.the.robot.s.decisions."      # (R)
    ),
    patient_impatient = c(
      "Did.the.robot.wait.for.you.before.acting.",
      "Did.the.robot.s.behavior.feel.rushed.",  # (R)
      "Did.the.robot.give.you.enough.time.to.complete.your.part.of.the.task.",
      "Did.the.robot.seem.willing.to.accommodate.your.pace."
    )
  )
  
  reversed_questions <- c(
    "Did.the.robot.seem.to.adapt.to.your.actions.or.follow.your.lead.",
    "Did.you.feel.like.you.had.to.adjust.to.the.robot.s.decisions.",
    "Did.the.robot.s.behavior.feel.rushed."
  )
  
  all_axis_questions <- unlist(axis_columns)
  
  # Check if required columns exist
  missing_columns <- setdiff(all_axis_questions, colnames(form_data))
  if (length(missing_columns) > 0) {
    warning("Missing axis-related columns: ", paste(missing_columns, collapse = ", "), 
            ". Filling axis scores with NA.")
    return(list(
      Leader_Follower = NA_real_,
      Patient_Impatient = NA_real_
    ))
  }
  
  # Helper function to map scores
  map_score <- function(score, reverse = FALSE) {
    adjusted <- score - 4
    if (reverse) adjusted <- -adjusted
    return(adjusted)
  }
  
  # Process scores for each axis. If no data rows exist, return NA.
  process_scores <- function(columns, reverse_scores = FALSE) {
    if (nrow(form_data) == 0) {
      return(NA_real_)
    }
    is_reversed <- columns %in% reversed_questions
    scores <- mapply(function(column, reverse) {
      val <- form_data[[column]]
      if (length(val) == 0) {
        return(rep(NA_real_, times = nrow(form_data)))
      }
      map_score(val, reverse)
    }, columns, is_reversed, SIMPLIFY = FALSE)
    
    scores_matrix <- do.call(cbind, scores)
    if (is.null(dim(scores_matrix))) {
      return(NA_real_)
    }
    
    row_means_values <- rowMeans(scores_matrix, na.rm = TRUE)
    # Reverse the direction of scores for Patient-Impatient axis
    final_score <- mean(row_means_values, na.rm = TRUE)
    if (reverse_scores) {
      final_score <- -final_score
    }
    return(final_score)
  }
  
  leader_follower_score <- process_scores(axis_columns$leader_follower)
  patient_impatient_score <- process_scores(axis_columns$patient_impatient, reverse_scores = TRUE)
  
  return(list(
    Leader_Follower = leader_follower_score,
    Patient_Impatient = patient_impatient_score
  ))
}


# Analyze logs and compute metrics from a given log file.
analyze_logs <- function(log_file) {
  results <- read.csv(log_file)
  
  # Ensure `total_reward` is numeric; convert if necessary
  if ("total_reward" %in% colnames(results)) {
    results$total_reward <- suppressWarnings(as.numeric(results$total_reward))
  } else {
    stop("Column 'total_reward' not found in log file: ", log_file)
  }
  
  # Required columns for strategy analysis
  required_columns <- c("strategy_phase_1", "strategy_phase_2", "strategy_phase_3")
  missing_columns <- setdiff(required_columns, colnames(results))
  
  if (length(missing_columns) > 0) {
    warning(
      "Missing columns in log file: ", 
      paste(missing_columns, collapse = ", "),
      ". Strategy analysis will be skipped."
    )
    strategy_changes <- NA
    stability <- NA
  } else {
    # Combine all phases into one "joint" strategy label per episode
    results$joint_strategy <- paste(
      results$strategy_phase_1,
      results$strategy_phase_2,
      results$strategy_phase_3,
      sep = "-"
    )
    
    # Count how many times strategy changes from one episode to the next
    # (same as before)
    strategy_changes <- sum(diff(as.numeric(factor(results$joint_strategy))) != 0)
    
    # -- NEW stability definition --
    # For each episode i (from the 2nd onward), check if it matches the previous one.
    # We'll say an episode is "stable" if it has the same strategy as episode i-1.
    
    # The first episode has no previous episode, so let's just mark it stable by default (count = 1).
    stable_episodes <- 1 + sum(
      results$joint_strategy[-1] == results$joint_strategy[-nrow(results)]
    )
    
    # Then define "stability" as fraction of episodes that are stable
    stability <- stable_episodes / nrow(results)
  }
  
  # Compute performance metrics
  total_episodes <- nrow(results)
  successful_episodes <- sum(results$task_status %in% c(0, 1), na.rm = TRUE)
  MPR <- (successful_episodes / total_episodes) * 100
  cumulative_reward <- sum(results$total_reward, na.rm = TRUE)
  
  list(
    Mean_Performance_Rate = MPR,
    Cumulative_Reward = cumulative_reward,
    Total_Strategy_Changes = strategy_changes,
    Stability = stability
  )
}



# Analyze all personality folders for a single participant
analyze_participant <- function(participant_dir) {
  # List subfolders (each representing a personality), but exclude "baseline" or "Baseline"
  personality_folders <- list.dirs(participant_dir, recursive = FALSE)
  personality_folders <- personality_folders[!tolower(basename(personality_folders)) %in% c("personality_type_baseline")]
  
  # For storing results
  results_list <- lapply(personality_folders, function(folder) {
    log_file <- file.path(folder, "logs", "episode_logs.csv")
    form_file <- file.path(folder, "form_responses.csv")
    
    if (!file.exists(log_file)) {
      # If there's no log file, skip
      return(NULL)
    }
    
    metrics <- analyze_logs(log_file) %>% as.data.frame()
    metrics$Participant <- basename(participant_dir)
    metrics$Personality <- basename(folder)
    
    # If form file exists, compute fluency & axis scores
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
    
    # Reorder columns: Put Participant, Personality first
    metrics <- metrics %>% dplyr::relocate(Participant, Personality)
    return(metrics)
  })
  
  # Bind all results into a single data frame
  do.call(dplyr::bind_rows, results_list)
}



# Updated analyze_all_participants function
analyze_all_participants <- function(data_collection_dir) {
  # Path to the co_learning_summary.csv file
  summary_file <- file.path(data_collection_dir, "co_learning_summary.csv")
  
  # Read the co_learning_summary.csv file
  co_learning_summary <- read.csv(summary_file)
  
  # Filter participants where Co_Learning_Occurred is TRUE
  valid_participants <- co_learning_summary %>%
    filter(Co_Learning_Occurred == TRUE) %>%
    pull(Participant)
  
  # Get full paths for valid participant directories
  participant_folders <- list.dirs(data_collection_dir, recursive = FALSE)
  valid_folders <- participant_folders[basename(participant_folders) %in% valid_participants]
  
  # Analyze only the valid participant folders
  all_results_list <- lapply(valid_folders, analyze_participant)
  
  # Combine results into a single data frame
  all_results <- bind_rows(all_results_list)
  
  return(all_results)
}

# Main script execution
final_results <- analyze_all_participants(data_collection_dir)

# 1) Read the Q-table metrics
q_table_metrics_path <- file.path(data_collection_dir, "q_table_metrics.csv")
if (!file.exists(q_table_metrics_path)) {
  warning("q_table_metrics.csv not found at: ", q_table_metrics_path)
} else {
  q_table_metrics <- read_csv(q_table_metrics_path)
  
  # 2) Merge with final_results by (Participant, Personality)
  #    - left_join() will retain all rows from final_results 
  #      and add matching columns from q_table_metrics
  final_results <- final_results %>%
    left_join(q_table_metrics, by = c("Participant", "Personality"))
}

# Print and write final results
print(final_results)

output_file <- file.path(data_collection_dir, "summary_metrics.csv")
write_csv(final_results, output_file)

message("Filtered summary metrics (including Q-table metrics) have been written to: ", output_file)


