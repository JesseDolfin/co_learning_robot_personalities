# Example: Encapsulated function for RM-ANOVA & post-hoc tests
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

# Usage example for all DVs:
dependent_vars <- c(
  "Mean_Performance_Rate", "Cumulative_Reward", "Total_Strategy_Changes",
  "Stability", "Fluency_Score", "Patient_Impatient_Score", "Leader_Follower_Score",
  "Avg_Entropy", "Avg_QGap", "Avg_Convergence", "Avg_ActionConsistency"
)

anova_results <- list()

for (dv in dependent_vars) {
  anova_results[[dv]] <- run_rm_anova(data, dv)
}

# Similarly, you can encapsulate your MANOVA in a function:
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

