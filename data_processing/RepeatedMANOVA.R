# =======================
# Repeated Measures ANOVA & Post-hoc Tests for Main DVs
# =======================
run_rm_anova <- function(data, dv, id_col = "Participant", within = "Personality") {
  library(afex)
  library(emmeans)
  
  cat("\n=======================================\n")
  cat("DV:", dv, "\n")
  cat("=======================================\n")
  
  # Run repeated-measures ANOVA
  aov_result <- aov_ez(
    id     = id_col,
    dv     = dv,
    data   = data,
    within = within,
    type   = 3
  )
  
  print(aov_result)
  
  # Extract p-value from the ANOVA table
  p_value <- aov_result$anova_table$`Pr(>F)`[1]
  
  # Run post-hoc tests if significant
  if (p_value < 0.05) {
    cat("\nPost-hoc tests for DV:", dv, "\n")
    posthoc <- emmeans(aov_result, pairwise ~ Personality, adjust = "tukey")
    print(posthoc)
  } else {
    cat("\nNo significant effect for DV:", dv, "(p =", p_value, ")\n")
  }
  
  return(aov_result)
}

# Example usage for main dependent variables:
dependent_vars <- c(
  "Mean_Performance_Rate", "Cumulative_Reward", "Total_Strategy_Changes",
  "Stability", "Fluency_Score", "Patient_Impatient_Score", "Leader_Follower_Score",
  "Avg_Entropy", "Avg_QGap", "Avg_Convergence", "Avg_ActionConsistency"
)

anova_results <- list()
for (dv in dependent_vars) {
  anova_results[[dv]] <- run_rm_anova(data, dv)
}

# =======================
# MANOVA Analysis
# =======================
run_manova <- function(data, dependent_vars) {
  library(tidyverse)
  library(car)
  
  # Pivot data into wide format
  wide_data <- data %>%
    pivot_wider(names_from = Personality, values_from = dependent_vars, names_sep = "_")
  
  # Create response matrix
  response_matrix <- as.matrix(wide_data %>%
                                 select(matches(paste(dependent_vars, collapse = "|"))))
  
  # Define idata for repeated-measures design
  idata <- expand.grid(
    DV = dependent_vars,
    Personality = levels(data$Personality)
  )
  
  # Fit MANOVA model
  manova_model <- lm(response_matrix ~ 1)
  manova_results <- Anova(
    manova_model, 
    idata = idata,
    idesign = ~ Personality, 
    type = "III"
  )
  
  cat("\nMANOVA Results (Multivariate Tests):\n")
  print(summary(manova_results, multivariate = TRUE))
  
  return(manova_results)
}

manova_results <- run_manova(data, dependent_vars)

# =======================
# New: Repeated Measures ANOVA for Human Perception Metrics
# (using human_perception_metrics.csv)
# =======================
run_human_perception_anova <- function(data, dv, id_col = "Participant", within = "Personality") {
  library(afex)
  library(emmeans)
  
  cat("\n=======================================\n")
  cat("Human Perception DV:", dv, "\n")
  cat("=======================================\n")
  
  # Run repeated-measures ANOVA on the human perception data
  aov_result <- aov_ez(
    id     = id_col,
    dv     = dv,
    data   = data,
    within = within,
    type   = 3
  )
  
  print(aov_result)
  
  # Extract p-value
  p_value <- aov_result$anova_table$`Pr(>F)`[1]
  
  # Run post-hoc tests if significant
  if (p_value < 0.05) {
    cat("\nPost-hoc tests for Human Perception DV:", dv, "\n")
    posthoc <- emmeans(aov_result, pairwise ~ Personality, adjust = "tukey")
    print(posthoc)
  } else {
    cat("\nNo significant effect for Human Perception DV:", dv, "(p =", p_value, ")\n")
  }
  
  return(aov_result)
}

# Usage example for human perception metrics:
# Define the dependent variables available in human_perception_metrics.csv
human_perception_dvs <- c(
  "Collaboration_Fluency", "Relative_Contribution", "Trust_in_Robot",
  "Positive_Teammate_Traits", "Perception_of_Improvement", "Perception_of_Shared_Goal"
)

# Assume data_collection_dir is defined and human_perception_metrics.csv is in that directory.
human_perception_file <- file.path(data_collection_dir, "human_perception_metrics.csv")

if (file.exists(human_perception_file)) {
  library(readr)
  human_perception_data <- read_csv(human_perception_file)
  
  human_perception_anova_results <- list()
  for (dv in human_perception_dvs) {
    human_perception_anova_results[[dv]] <- run_human_perception_anova(human_perception_data, dv)
  }
} else {
  warning("human_perception_metrics.csv not found at: ", human_perception_file)
}
