# Load necessary libraries
library(dplyr)
library(purrr)
library(readr)

# Optional: Set working directory to the script's location
if (requireNamespace("rstudioapi", quietly = TRUE)) {
  setwd(dirname(rstudioapi::getActiveDocumentContext()$path))
}

# Define the data collection directory
data_collection_dir <- "/home/jesse/thesis/src/co_learning_robot_personalities/data_collection"

# Debug: Check the folder path
message("Data collection directory: ", data_collection_dir)
message("Participant folders found: ", paste(list.files(data_collection_dir), collapse = ", "))

# ------------------------------------------------------------------------------
# Helper: Calculate the single 'fluency' score (existing approach)
# ------------------------------------------------------------------------------
calculate_fluency <- function(form_file) {
  form_data <- read.csv(form_file)
  
  # The original list of columns used to calculate "fluency"
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
    stop("Missing columns for fluency calculation: ", paste(missing, collapse = ", "))
  }
  
  fluency_scores <- form_data %>%
    dplyr::select(dplyr::all_of(fluency_columns)) %>%
    rowMeans(na.rm = TRUE)
  
  return(mean(fluency_scores, na.rm = TRUE))
}

# ------------------------------------------------------------------------------
# Helper: Calculate "leader-follower" & "patient-impatient" axes (existing)
# ------------------------------------------------------------------------------
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
  
  missing_columns <- setdiff(all_axis_questions, colnames(form_data))
  if (length(missing_columns) > 0) {
    warning("Missing axis-related columns: ", 
            paste(missing_columns, collapse = ", "),
            ". Will set axis scores to NA.")
    return(list(
      Leader_Follower = NA_real_,
      Patient_Impatient = NA_real_
    ))
  }
  
  map_score <- function(score, reverse = FALSE) {
    # Example: center around 4, etc. (Your original approach)
    adjusted <- score - 4
    if (reverse) adjusted <- -adjusted
    return(adjusted)
  }
  
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

# ------------------------------------------------------------------------------
# Helper: Compute performance metrics from logs (existing)
# ------------------------------------------------------------------------------
analyze_logs <- function(log_file) {
  results <- read.csv(log_file)
  
  # Ensure total_reward is numeric
  if ("total_reward" %in% colnames(results)) {
    results$total_reward <- suppressWarnings(as.numeric(results$total_reward))
  } else {
    stop("Column 'total_reward' not found in log file: ", log_file)
  }
  
  # Strategy analysis: check for missing columns
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
    # Combine all phases into one "joint" strategy label
    results$joint_strategy <- paste(
      results$strategy_phase_1,
      results$strategy_phase_2,
      results$strategy_phase_3,
      sep = "-"
    )
    
    strategy_changes <- sum(diff(as.numeric(factor(results$joint_strategy))) != 0)
    
    # Compute "stability" as fraction of consecutive episodes with unchanged strategy
    stable_episodes <- 1 + sum(
      results$joint_strategy[-1] == results$joint_strategy[-nrow(results)]
    )
    stability <- stable_episodes / nrow(results)
  }
  
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

# ------------------------------------------------------------------------------
# (EXISTING) Analyze a single participant folder for logs & optional form data
# ------------------------------------------------------------------------------
analyze_participant <- function(participant_dir) {
  personality_folders <- list.dirs(participant_dir, recursive = FALSE)
  # Exclude baseline folder
  personality_folders <- personality_folders[!tolower(basename(personality_folders)) %in% c("personality_type_baseline")]
  
  results_list <- lapply(personality_folders, function(folder) {
    log_file <- file.path(folder, "logs", "episode_logs.csv")
    form_file <- file.path(folder, "form_responses.csv")
    
    if (!file.exists(log_file)) {
      return(NULL)  # Skip if no logs
    }
    
    metrics <- analyze_logs(log_file) %>% as.data.frame()
    metrics$Participant <- basename(participant_dir)
    metrics$Personality <- basename(folder)
    
    # If form file exists, compute existing "fluency" + axis scores
    if (file.exists(form_file)) {
      metrics$Fluency_Score <- calculate_fluency(form_file)
      axis_scores <- calculate_axis_scores(form_file)
      metrics$Patient_Impatient_Score <- axis_scores$Patient_Impatient
      metrics$Leader_Follower_Score   <- axis_scores$Leader_Follower
    } else {
      metrics$Fluency_Score <- NA
      metrics$Patient_Impatient_Score <- NA
      metrics$Leader_Follower_Score   <- NA
    }
    
    # Reorder columns
    metrics <- metrics %>% dplyr::relocate(Participant, Personality)
    return(metrics)
  })
  
  do.call(dplyr::bind_rows, results_list)
}

calculate_human_perception_scores <- function(form_file) {
  # Map your 15 statements to the EXACT trimmed column names in the CSV
  question_col_map <- c(
    Q1  = "The.robot.improved.over.time.",
    Q2  = "The.team.worked.fluently.together.",
    Q3  = "The.robot.adapted.to.my.input.as.the.task.progressed.",
    Q4  = "The.robot.contributed.to.the.team.s.success",
    Q5  = "I.trusted.the.robot.to.act.according.to.our.shared.goals.",
    Q6  = "The.robot.made.decisions.that.aligned.with.my.expectations.for.timing.and.handover.actions.",
    Q7  = "The.robot.was.committed.to.the.task.",
    Q8  = "I.felt.like.an.equal.partner.in.the.team.",
    Q9  = "I.had.to.guide.the.robot.more.than.expected.",
    Q10 = "The.robot.demonstrated.an.understanding.of.the.shared.task.goals.",
    Q11 = "I.adjusted.my.actions.based.on.the.robot.s.behavior.",
    Q12 = "The.robot.made.independent.decisions.when.appropriate.",
    Q13 = "I.had.to.constantly.monitor.the.robot.s.actions.to.ensure.task.success.",
    Q14 = "The.robot.and.I.shared.a.mutual.understanding.of.the.task.requirements.",
    Q15 = "The.robot.made.independent.decisions.when.appropriate.to.support.the.task."
  )
  
  
  # Your 6 categories:
  categories <- list(
    Collaboration_Fluency     = c("Q2", "Q3", "Q4"),
    Relative_Contribution     = c("Q8", "Q9", "Q12"),
    Trust_in_Robot            = c("Q5", "Q13"),
    Positive_Teammate_Traits  = c("Q6", "Q7", "Q12", "Q15"),
    Perception_of_Improvement = c("Q1", "Q3", "Q11"),
    Perception_of_Shared_Goal = c("Q5", "Q10", "Q14")
  )
  
  # Reverse-scored question numbers:
  reversed_questions <- c("Q9", "Q13")
  
  # If there's no file, return all-NA row
  if (!file.exists(form_file)) {
    return(as.data.frame(as.list(setNames(rep(NA_real_, length(categories)), names(categories)))))
  }
  
  # Read CSV, trim column names, no factors
  form_data <- read.csv(form_file, stringsAsFactors = FALSE)
  colnames(form_data) <- trimws(colnames(form_data))
  

  if (nrow(form_data) == 0) {
    # If CSV is empty, all NA
    return(as.data.frame(as.list(setNames(rep(NA_real_, length(categories)), names(categories)))))
  }
  
  # Collect the needed columns (unique list of all Q1..Q15)
  needed_cols <- unique(unname(question_col_map))
  # Check for missing columns
  missing_cols <- setdiff(needed_cols, colnames(form_data))
  if (length(missing_cols) > 0) {
    warning("Missing question columns in 'form_responses.csv': ",
            paste(missing_cols, collapse = ", "),
            "\nReturning NA for all categories.")
    return(as.data.frame(as.list(setNames(rep(NA_real_, length(categories)), names(categories)))))
  }
  
  # Force numeric conversion on each needed column
  for (colname in needed_cols) {
    form_data[[colname]] <- suppressWarnings(as.numeric(form_data[[colname]]))
  }
  
  # Subset the form_data to only the needed columns to avoid coercion issues
  form_data_needed <- form_data[needed_cols]
  
  # Helper for single question
  get_question_score <- function(q_key, row_vals) {
    col_name <- question_col_map[[q_key]]
    raw_score <- row_vals[[col_name]]
    if (is.na(raw_score)) return(NA_real_)
    if (q_key %in% reversed_questions) {
      return(8 - raw_score)
    } else {
      return(raw_score)
    }
  }
  
  # Now apply over the subsetted, all-numeric data
  category_scores_per_row <- apply(form_data_needed, 1, function(row_i) {
    row_i <- as.list(row_i)
    sapply(categories, function(q_keys) {
      these_scores <- sapply(q_keys, get_question_score, row_vals = row_i)
      mean(as.numeric(these_scores), na.rm = TRUE)
    })
  })
  
  # We might have multiple rows in form_responses.csv. 
  # category_scores_per_row is either a matrix [category x rows] or a vector [category].
  if (is.matrix(category_scores_per_row)) {
    # If there's more than 1 response row, we average across those rows for each category.
    # So each category is a row in category_scores_per_row:
    # -> apply(..., 1, mean) to average across columns (the multiple responses).
    if (ncol(category_scores_per_row) > 1) {
      final_scores <- apply(category_scores_per_row, 1, mean, na.rm = TRUE)
    } else {
      # Only one row => single column => just drop to a vector
      final_scores <- category_scores_per_row[, 1]
    }
  } else {
    # Single row, so it's already a category vector
    final_scores <- category_scores_per_row
  }
  
  final_scores_df <- as.data.frame(as.list(final_scores))
  # Name the columns by category
  names(final_scores_df) <- names(categories)
  
  return(final_scores_df)
}



# ------------------------------------------------------------------------------
# (NEW) Analyze participant folder for the “human perception” categories only
# ------------------------------------------------------------------------------
analyze_participant_perception <- function(participant_dir) {
  personality_folders <- list.dirs(participant_dir, recursive = FALSE)
  # Exclude baseline
  personality_folders <- personality_folders[!tolower(basename(personality_folders)) %in% c("personality_type_baseline")]
  
  # For each personality folder, if form_responses.csv exists, compute category-based scores
  df_list <- lapply(personality_folders, function(folder) {
    form_file <- file.path(folder, "form_responses.csv")
    
    # If there's no form file, we’ll just get NAs
    perception_scores <- calculate_human_perception_scores(form_file)
    # Add participant & personality columns
    perception_scores$Participant <- basename(participant_dir)
    perception_scores$Personality <- basename(folder)
    
    # Reorder columns to put Participant/Personality first
    perception_scores <- perception_scores %>%
      dplyr::relocate(Participant, Personality)
    
    return(perception_scores)
  })
  
  do.call(dplyr::bind_rows, df_list)
}

# ------------------------------------------------------------------------------
# (NEW) Top-level function: analyze all participants for human perception
# ------------------------------------------------------------------------------
analyze_all_participants_perception <- function(data_collection_dir) {
  summary_file <- file.path(data_collection_dir, "co_learning_summary.csv")
  co_learning_summary <- read.csv(summary_file)
  
  # Only participants where Co_Learning_Occurred is TRUE
  valid_participants <- co_learning_summary %>%
    filter(Co_Learning_Occurred == TRUE) %>%
    pull(Participant)
  
  participant_folders <- list.dirs(data_collection_dir, recursive = FALSE)
  valid_folders <- participant_folders[basename(participant_folders) %in% valid_participants]
  
  # Analyze each valid participant
  all_perception_results <- lapply(valid_folders, analyze_participant_perception)
  
  # Combine
  all_perception_results <- bind_rows(all_perception_results)
  return(all_perception_results)
}

# ------------------------------------------------------------------------------
# Main script execution
# ------------------------------------------------------------------------------
final_results <- analyze_all_participants(data_collection_dir)

# 1) Read the Q-table metrics
q_table_metrics_path <- file.path(data_collection_dir, "q_table_metrics.csv")
if (!file.exists(q_table_metrics_path)) {
  warning("q_table_metrics.csv not found at: ", q_table_metrics_path)
} else {
  q_table_metrics <- read_csv(q_table_metrics_path)
  final_results <- final_results %>%
    left_join(q_table_metrics, by = c("Participant", "Personality"))
}

# Write out the usual summary metrics
print(final_results)
output_file <- file.path(data_collection_dir, "summary_metrics.csv")
write_csv(final_results, output_file)
message("Filtered summary metrics (including Q-table metrics) have been written to: ", output_file)

# ------------------------------------------------------------------------------
# (NEW) Produce the separate category-based file: human_perception_metrics.csv
# ------------------------------------------------------------------------------
final_perception_results <- analyze_all_participants_perception(data_collection_dir)
print(final_perception_results)

perception_file <- file.path(data_collection_dir, "human_perception_metrics.csv")
write_csv(final_perception_results, perception_file)
message("Human perception category-based scores have been written to: ", perception_file)
