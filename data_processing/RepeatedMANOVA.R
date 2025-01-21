###############################################################################
# File: MANOVA_and_RM_ANOVA_no_FDR.R
#
# Purpose:
#   1) Perform a repeated-measures MANOVA across multiple metrics (DVs).
#   2) Follow up with:
#      - Repeated-measures ANOVAs for each metric.
#      - Post-hoc pairwise comparisons for metrics with significant \(p\)-values.
###############################################################################

# -----------------------------
# 0. Load Libraries
# -----------------------------
library(afex)         # for aov_ez() repeated-measures ANOVAs
library(emmeans)      # for post-hoc tests
library(car)          # for MANOVA with repeated-measures design
library(tidyverse)    # for data wrangling

# -----------------------------
# 1. Read and Prepare Data
# -----------------------------
data_file <- "/home/jesse/thesis/src/co_learning_robot_personalities/data_collection/summary_metrics.csv"
data <- read.csv(data_file, stringsAsFactors = FALSE)

# Convert relevant columns to factors
data$Participant <- factor(data$Participant)
data$Personality <- factor(data$Personality)

# Quick sanity checks
cat("Data preview:\n")
print(head(data))
cat("\nParticipant x Personality count:\n")
print(table(data$Participant, data$Personality))

# -----------------------------
# 2. Identify Dependent Variables
# -----------------------------
dependent_vars <- c(
  "Mean_Performance_Rate",
  "Cumulative_Reward",
  "Total_Strategy_Changes",
  "Stability",
  "Fluency_Score",
  "Patient_Impatient_Score",
  "Leader_Follower_Score",
  "Avg_Entropy",
  "Avg_QGap",
  "Avg_Convergence",
  "Avg_ActionConsistency"
)

# -----------------------------
# 3. Run MANOVA
# -----------------------------
cat("\n--- STEP 3: Running MANOVA ---\n")

# Pivot data into wide format
wide_data <- data %>%
  pivot_wider(names_from = Personality, values_from = dependent_vars, names_sep = "_")

# Create response matrix (DVs x Personality combinations)
response_matrix <- as.matrix(wide_data %>%
                               select(starts_with("Mean_Performance_Rate"), starts_with("Cumulative_Reward"), 
                                      starts_with("Total_Strategy_Changes"), starts_with("Stability"), 
                                      starts_with("Fluency_Score"), starts_with("Patient_Impatient_Score"), 
                                      starts_with("Leader_Follower_Score"), starts_with("Avg_Entropy"),
                                      starts_with("Avg_QGap"), starts_with("Avg_Convergence"), 
                                      starts_with("Avg_ActionConsistency")))

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

# Display MANOVA results
cat("\nMANOVA Results (Multivariate Tests):\n")
print(summary(manova_results, multivariate = TRUE))

# -----------------------------
# 4. Run Repeated-Measures ANOVAs
# -----------------------------
anova_results <- list()  # To store ANOVA results for all metrics

cat("\n--- STEP 4: Running Repeated-Measures ANOVAs ---\n")
for (dv in dependent_vars) {
  cat("\n=======================================\n")
  cat("DV:", dv, "\n")
  cat("=======================================\n")
  
  # Repeated-measures ANOVA
  aov_result <- aov_ez(
    id     = "Participant",
    dv     = dv,
    data   = data,
    within = "Personality",
    type   = 3
  )
  
  # Store the result
  anova_results[[dv]] <- aov_result
}

# -----------------------------
# 5. Post-Hoc Tests for Significant DVs
# -----------------------------
cat("\n--- STEP 5: Post-Hoc Tests for Significant DVs ---\n")
for (dv in dependent_vars) {
  # Extract p-value from the ANOVA table
  p_value <- anova_results[[dv]]$anova_table$`Pr(>F)`[1]
  
  # Perform post-hoc tests if p-value < 0.05
  if (p_value < 0.05) {
    cat("\nPost-hoc tests for DV:", dv, "\n")
    
    # Post-hoc comparisons
    posthoc <- emmeans(anova_results[[dv]], pairwise ~ Personality, adjust = "tukey")
    print(posthoc)
  } else {
    cat("\nNo significant effect for DV:", dv, "(p =", p_value, ")\n")
  }
}

cat("\n\n*** Analysis Complete! ***\n")
