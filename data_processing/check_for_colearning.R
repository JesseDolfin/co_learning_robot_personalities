# ------------------------------------------------------
# Final Complete Script (with Convergence Checks)
# ------------------------------------------------------

# Load necessary libraries
library(dplyr)
library(purrr)
library(readr)
library(kableExtra)

# Set the default thresholds
phase_stability_thres = 0.3
instability_threshold = 3
stability_threshold = 4

# ------------------------------------------------------
# Helper Functions
# ------------------------------------------------------

# 1) Calculate Fluency
calculate_fluency <- function(form_file) {
  form_data <- read.csv(form_file)
  
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
    stop("Missing fluency columns: ", paste(missing, collapse = ", "))
  }
  
  fluency_scores <- form_data %>%
    dplyr::select(dplyr::all_of(fluency_columns)) %>%
    rowMeans(na.rm = TRUE)
  
  return(mean(fluency_scores, na.rm = TRUE))
}

# 2) Calculate Axis Scores
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
            ". Filling axis scores with NA.")
    return(list(
      Leader_Follower = NA_real_,
      Patient_Impatient = NA_real_
    ))
  }
  
  map_score <- function(score, reverse = FALSE) {
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

# ------------------------------------------------------
# NEW HELPER: Fill Forward Missing Strategies
# ------------------------------------------------------
fill_forward <- function(x) {
  # Ensure x is an atomic vector
  if (!is.atomic(x)) {
    x <- as.character(x)
  }
  
  # Fill forward
  for (i in 2:length(x)) {
    if (is.na(x[i]) || x[i] == "") {
      x[i] <- x[i - 1]
    }
  }
  
  # Fill backward for leading NAs or empty strings
  for (i in (length(x) - 1):1) {
    if ((is.na(x[i]) || x[i] == "") && !is.na(x[i + 1]) && x[i + 1] != "") {
      x[i] <- x[i + 1]
    }
  }
  
  return(x)
}

# ------------------------------------------------------
# NEW HELPER: Check Convergence (instability followed by stability)
# ------------------------------------------------------
check_convergence <- function(strategies_vector) {
  # Remove NA values
  strategies_vector <- na.omit(strategies_vector)
  
  # If no valid strategies remain, return FALSE
  if (length(strategies_vector) < 3) {
    return(FALSE)
  }
  
  # Compute runs
  runs <- rle(strategies_vector)
  run_lengths <- runs$lengths
  
  # Safeguard: Ensure all elements in run_lengths are valid numbers
  if (any(is.na(run_lengths))) {
    return(FALSE)
  }
  
  # Check for instability followed by stability
  for (i in seq_along(run_lengths)) {
    if (run_lengths[i] <= instability_threshold) {
      # Check if there's any run *after* i that is >=3
      valid_run_after <- run_lengths[(i + 1):length(run_lengths)]
      if (any(valid_run_after > stability_threshold, na.rm = TRUE)) {
        return(TRUE)
      }
    }
  }
  
  return(FALSE)
}



# 3) Analyze Logs (Modified to Include Phase-by-Phase Changes & Convergence)
analyze_logs <- function(log_file) {
  # Read in logs
  results <- read.csv(log_file, stringsAsFactors = FALSE)
  
  # Ensure total_reward is numeric
  results$total_reward <- suppressWarnings(as.numeric(results$total_reward))
  
  # Fill forward missing strategies and ensure no unexpected values
  results$strategy_phase_1 <- fill_forward(results$strategy_phase_1)
  results$strategy_phase_2 <- fill_forward(results$strategy_phase_2)
  results$strategy_phase_3 <- fill_forward(results$strategy_phase_3)
  
  # Basic metrics
  total_episodes <- nrow(results)
  successful_episodes <- sum(results$task_status %in% c(0, 1), na.rm = TRUE)
  MPR <- (successful_episodes / total_episodes) * 100
  cumulative_reward <- sum(results$total_reward, na.rm = TRUE)
  
  # Check columns for strategies
  required_columns <- c("strategy_phase_1", "strategy_phase_2", "strategy_phase_3")
  missing_columns <- setdiff(required_columns, colnames(results))
  if (length(missing_columns) > 0) {
    warning(
      "Missing columns in log file: ", 
      paste(missing_columns, collapse = ", "),
      ". Strategy analysis (joint) will be skipped."
    )
    # Return partial results with placeholders
    return(list(
      Mean_Performance_Rate = MPR,
      Cumulative_Reward     = cumulative_reward,
      Total_Strategy_Changes = NA,
      Stability             = NA,
      Phase1_Changes        = NA,
      Phase2_Changes        = NA,
      Phase3_Changes        = NA,
      Phase_Stability       = NA,
      # NEW placeholders for convergence columns
      Convergence_Phase_1   = NA,
      Convergence_Phase_2   = NA,
      Convergence_Phase_3   = NA
    ))
  }
  
  # Compute joint strategy
  results$joint_strategy <- paste(
    results$strategy_phase_1,
    results$strategy_phase_2,
    results$strategy_phase_3,
    sep = "-"
  )
  
  strategy_changes <- sum(diff(as.numeric(factor(results$joint_strategy))) != 0)
  stable_episodes <- sum(results$joint_strategy == results$joint_strategy[1])
  overall_stability <- stable_episodes / nrow(results)
  
  # Fill-forward missing strategies for each phase
  phase1 <- fill_forward(results$strategy_phase_1)
  phase2 <- fill_forward(results$strategy_phase_2)
  phase3 <- fill_forward(results$strategy_phase_3)
  
  print(phase2)
  
  # Count changes for each phase
  phase1_changes <- 0
  phase2_changes <- 0
  phase3_changes <- 0
  if (total_episodes > 1) {
    phase1_changes <- sum(phase1[-1] != phase1[-total_episodes], na.rm = TRUE)
    phase2_changes <- sum(phase2[-1] != phase2[-total_episodes], na.rm = TRUE)
    phase3_changes <- sum(phase3[-1] != phase3[-total_episodes], na.rm = TRUE)
  }
  
  # Original "Phase_Stability" was fraction of episodes not changing in ANY phase
  if (total_episodes > 1) {
    stable_transition_flags <- mapply(function(i) {
      (phase1[i] == phase1[i-1]) &&
        (phase2[i] == phase2[i-1]) &&
        (phase3[i] == phase3[i-1])
    },
    2:total_episodes)
    phase_stability <- mean(stable_transition_flags)
  } else {
    phase_stability <- NA_real_
  }
  
  # ------------------------------------------------------
  # NEW: Compute whether each phase converged
  # ------------------------------------------------------
  convergence_phase_1 <- check_convergence(phase1)
  convergence_phase_2 <- check_convergence(phase2)
  convergence_phase_3 <- check_convergence(phase3)
  
  # Return results (remove old Phase1_Stability and Phase2_Stability)
  return(list(
    Mean_Performance_Rate   = MPR,
    Cumulative_Reward       = cumulative_reward,
    Total_Strategy_Changes  = strategy_changes,
    Stability               = overall_stability,
    Phase1_Changes          = phase1_changes,
    Phase2_Changes          = phase2_changes,
    Phase3_Changes          = phase3_changes,
    Phase_Stability         = phase_stability,
    # NEW booleans for each phase's convergence
    Convergence_Phase_1     = convergence_phase_1,
    Convergence_Phase_2     = convergence_phase_2,
    Convergence_phase_3     = convergence_phase_3
  ))
}

# 4) Analyze a Single Participant
analyze_participant <- function(participant_dir) {
  # List subfolders (exclude baseline)
  personality_folders <- list.dirs(participant_dir, recursive = FALSE)
  personality_folders <- personality_folders[
    !tolower(basename(personality_folders)) %in% c("personality_type_baseline")
  ]
  
  results_list <- lapply(personality_folders, function(folder) {
    log_file <- file.path(folder, "logs", "episode_logs.csv")
    form_file <- file.path(folder, "form_responses.csv")
    
    folder_name <- tolower(basename(folder))
    # Remove the "personality_type_" prefix if present:
    personality <- sub("^personality_type_", "", folder_name)
    
    if (!file.exists(log_file)) {
      return(NULL)
    }
    
    metrics <- analyze_logs(log_file) %>% as.data.frame()
    metrics$Participant <- basename(participant_dir)
    metrics$Personality <- personality
    
    # If a form file exists, compute Fluency & Axis scores
    if (file.exists(form_file)) {
      metrics$Fluency_Score <- calculate_fluency(form_file)
      axis_scores <- calculate_axis_scores(form_file)
      metrics$Patient_Impatient_Score <- axis_scores$Patient_Impatient
      metrics$Leader_Follower_Score   <- axis_scores$Leader_Follower
    } else {
      metrics$Fluency_Score           <- NA
      metrics$Patient_Impatient_Score <- NA
      metrics$Leader_Follower_Score   <- NA
    }
    
    # Reorder columns: Participant, Personality first
    metrics <- metrics %>% dplyr::relocate(Participant, Personality)
    return(metrics)
  })
  
  # Combine results
  do.call(dplyr::bind_rows, results_list)
}

# ------------------------------------------------------
# Latin Square Definition
# ------------------------------------------------------
latin_square <- list(
  c("follower",  "impatient", "leader",   "patient"),    
  c("follower",  "impatient", "patient",  "leader"),    
  c("follower",  "leader",    "impatient","patient"),    
  c("follower",  "patient",   "leader",   "impatient"),    
  c("impatient", "follower",  "leader",   "patient"),    
  c("impatient", "leader",    "patient",  "follower"),    
  c("impatient", "patient",   "follower", "leader"),    
  c("impatient", "patient",   "leader",   "follower"),    
  c("leader",    "follower",  "impatient","patient"),    
  c("leader",    "impatient", "patient",  "follower"),    
  c("leader",    "patient",   "follower", "impatient"),    
  c("leader",    "patient",   "impatient","follower"),    
  c("patient",   "follower",  "impatient","leader"),    
  c("patient",   "impatient", "follower", "leader"),    
  c("patient",   "leader",    "follower", "impatient"),    
  c("patient",   "leader",    "impatient","follower"),    
  c("follower",  "leader",    "patient",  "impatient"),    
  c("follower",  "patient",   "impatient","leader"),    
  c("impatient", "follower",  "patient",  "leader"),    
  c("impatient", "leader",    "follower", "patient"),    
  c("leader",    "follower",  "patient",  "impatient"),    
  c("leader",    "impatient", "follower", "patient"),    
  c("patient",   "follower",  "leader",   "impatient"),    
  c("patient",   "impatient", "leader",   "follower")     
)

# ------------------------------------------------------
# Analyze All Participants
# ------------------------------------------------------
analyze_all_participants <- function(data_collection_dir,
                                     latin_square,
                                     strategy_threshold = 6) {
  participant_folders <- list.dirs(data_collection_dir, recursive = FALSE)
  
  # Summarize at Participant Level
  co_learning_summary <- data.frame(
    Participant               = character(),
    Fluency_Improvement       = numeric(),
    MPR_Improvement           = numeric(),
    Avg_Phase_Stability       = numeric(),
    # Keep these for reference if needed
    Convergence_Phase_1_Any   = logical(),
    Convergence_Phase_2_Any   = logical(),
    Convergence_Phase_3_Any   = logical(),
    Co_Learning_Occurred      = logical(),
    stringsAsFactors          = FALSE
  )
  
  for (i in seq_along(participant_folders)) {
    participant_dir  <- participant_folders[i]
    participant_name <- basename(participant_dir)
    
    # Attempt to parse a numeric ID from folder name or fallback to i
    participant_index <- suppressWarnings(as.numeric(gsub("\\D+", "", participant_name)))
    if (is.na(participant_index) || 
        participant_index < 1 || 
        participant_index > length(latin_square)) {
      participant_index <- i
    }
    
    participant_data <- analyze_participant(participant_dir)
    
    if (is.null(participant_data) || nrow(participant_data) == 0) {
      next
    }
    
    # Reorder the data by the Latin square
    correct_order <- latin_square[[participant_index]]
    ordered_data <- participant_data %>%
      mutate(Personality = tolower(Personality)) %>%
      filter(Personality %in% correct_order) %>%
      mutate(Personality = factor(Personality, levels = correct_order)) %>%
      arrange(Personality)
    
    # Compute average (overall) phase stability
    avg_phase_stability <- mean(ordered_data$Phase_Stability, na.rm = TRUE)
    
    # Compute fluency improvement (difference between last and first condition)
    fluency_improvement <- NA_real_
    if (nrow(ordered_data) >= 2) {
      fluency_improvement <- ordered_data$Fluency_Score[nrow(ordered_data)] -
        ordered_data$Fluency_Score[1]
    }
    
    # Compute MPR improvement (difference between last and first condition)
    mpr_improvement <- NA_real_
    if (nrow(ordered_data) >= 2) {
      mpr_improvement <- ordered_data$Mean_Performance_Rate[nrow(ordered_data)] -
        ordered_data$Mean_Performance_Rate[1]
    }
    
    # Check if Phase 1 or Phase 2 converged in ANY of the personality runs
    phase1_convergence_any <- any(ordered_data$Convergence_Phase_1 == TRUE, na.rm = TRUE)
    phase2_convergence_any <- any(ordered_data$Convergence_Phase_2 == TRUE, na.rm = TRUE)
    phase3_convergence_any <- any(ordered_data$Convergence_Phase_3 == TRUE, na.rm = TRUE)
    
    # Strategy changes average across all runs (just if you still want it)
    avg_strategy_changes <- mean(ordered_data$Total_Strategy_Changes, na.rm = TRUE)
    
    # Define co-learning:
    #   You can incorporate the new convergence booleans here:
    co_learning_occurred <- sum(
      !is.na(fluency_improvement) && fluency_improvement > 0,
      !is.na(mpr_improvement) && mpr_improvement > 0,
      avg_phase_stability > phase_stability_thres,
      phase1_convergence_any || phase2_convergence_any || phase3_convergence_any
    ) >= 2
    
    
    # Append a row
    co_learning_summary <- rbind(
      co_learning_summary,
      data.frame(
        Participant              = participant_name,
        Fluency_Improvement      = fluency_improvement,
        MPR_Improvement          = mpr_improvement,
        Avg_Phase_Stability      = avg_phase_stability,
        Convergence_Phase_1_Any  = phase1_convergence_any,
        Convergence_Phase_2_Any  = phase2_convergence_any,
        Convergence_Phase_3_Any  = phase3_convergence_any,
        Co_Learning_Occurred     = co_learning_occurred,
        stringsAsFactors         = FALSE
      )
    )
  }
  
  return(co_learning_summary)
}

# ------------------------------------------------------
# Main Execution
# ------------------------------------------------------

# 1) Set your data collection directory
data_collection_dir <- "/home/jesse/thesis/src/co_learning_robot_personalities/data_collection"

# 2) Run the participant-level analysis
final_table <- analyze_all_participants(
  data_collection_dir = data_collection_dir,
  latin_square        = latin_square,
  strategy_threshold  = -1
)

# 3) Round numeric columns to 2 decimals
final_table <- final_table %>%
  mutate(
    Fluency_Improvement  = round(Fluency_Improvement, 2),
    MPR_Improvement      = round(MPR_Improvement, 2),
    Avg_Phase_Stability  = round(Avg_Phase_Stability, 2)
  )

# 4) In the final table, we remove any columns you no longer want
#    (Here, we already replaced Phase1_Stability/Phase2_Stability
#     with the convergence booleans, so those are no longer included.)
#    We keep Convergence_Phase_1_Any and Convergence_Phase_2_Any.
#    Remove if you had any old columns leftover.
#    For example, if you don't want to show "Avg_Phase_Stability" either,
#    you could remove it. Below shows how if needed:
colnames(final_table)
final_table <- final_table %>%
   select(-Convergence_Phase_3_Any)


# 5) Print the table
print(final_table)

# 6) Write it to CSV
output_file <- file.path(data_collection_dir, "co_learning_summary.csv")
write_csv(final_table, output_file)
message("Co-learning summary has been written to: ", output_file)

# 7) Create a color-coded table with kableExtra
color_coded_table <- final_table %>%
  mutate(
    Fluency_Improvement = cell_spec(
      Fluency_Improvement, "html",
      color = ifelse(
        is.na(Fluency_Improvement), 
        "black", 
        ifelse(Fluency_Improvement > 0, "green", "red")
      )
    ),
    MPR_Improvement = cell_spec(
      MPR_Improvement, "html",
      color = ifelse(
        is.na(MPR_Improvement), 
        "black", 
        ifelse(MPR_Improvement > 0, "green", "red")
      )
    ),
    Avg_Phase_Stability = cell_spec(
      Avg_Phase_Stability, "html",
      color = ifelse(
        is.na(Avg_Phase_Stability),
        "black",
        ifelse(Avg_Phase_Stability > phase_stability_thres, "green", "red")
      )
    ),
    Convergence_Phase_1_Any = cell_spec(
      Convergence_Phase_1_Any, "html",
      color = ifelse(
        is.na(Convergence_Phase_1_Any), 
        "black", 
        ifelse(Convergence_Phase_1_Any == TRUE, "green", "red")
      )
    ),
    Convergence_Phase_2_Any = cell_spec(
      Convergence_Phase_2_Any, "html",
      color = ifelse(
        is.na(Convergence_Phase_2_Any), 
        "black", 
        ifelse(Convergence_Phase_2_Any == TRUE, "green", "red")
      )
    ),
    Co_Learning_Occurred = cell_spec(
      Co_Learning_Occurred, "html",
      color = ifelse(
        is.na(Co_Learning_Occurred), 
        "black", 
        ifelse(Co_Learning_Occurred == TRUE, "green", "red")
      )
    )
  ) %>%
  kable("html", escape = FALSE) %>%
  kable_styling("striped", full_width = FALSE)

# 8) Display the color-coded table in an interactive environment
color_coded_table
