# -------------------------------------------------------------------------
# ANNOVA.R
#
# Purpose:
# Perform ANOVA on Mean Performance Rate (MPR) across different personalities.
# Post-hoc comparisons are also conducted.
#
# Requirements:
# - The dataset "summary_metrics_all_participants.csv" should exist in
#   data_collection_dir.
# - The dataset should contain "Mean_Performance_Rate", "Personality", 
#   and "Participant" columns.
#
# Notes:
# - Ensure assumptions for ANOVA are met (normality, homoscedasticity).
# - Consider transformations or non-parametric tests if assumptions are violated.
# -------------------------------------------------------------------------

# Load necessary libraries
library(dplyr)
library(emmeans)

# Define file paths and directories
data_collection_dir <- "/home/jesse/thesis/src/co_learning_robot_personalities/data_collection"
data_file <- file.path(data_collection_dir, "summary_metrics_all_participants.csv")

# Check if the file exists
if (!file.exists(data_file)) {
  stop("Data file not found at: ", data_file)
}

# Load the aggregated data
data <- read.csv(data_file)

# Check that required columns exist
required_cols <- c("Mean_Performance_Rate", "Personality", "Participant")
missing_cols <- setdiff(required_cols, colnames(data))
if (length(missing_cols) > 0) {
  stop("The following required columns are missing: ", paste(missing_cols, collapse = ", "))
}

# Convert Personality and Participant to factors
data <- data %>%
  mutate(Personality = as.factor(Personality),
         Participant = as.factor(Participant))

# Debug: Check a sample of the data and structure
message("Preview of the data:")
print(head(data))
str(data)
summary(data)

# Perform repeated-measures ANOVA for Mean_Performance_Rate
# Using Error(Participant/Personality) to treat Participant as a random factor
anova_mpr <- aov(Mean_Performance_Rate ~ Personality + Error(Participant/Personality), data = data)
message("ANOVA summary for Mean_Performance_Rate:")
print(summary(anova_mpr))

# Post-hoc tests for Mean_Performance_Rate using emmeans
mpr_posthoc <- emmeans(anova_mpr, pairwise ~ Personality)
message("Post-hoc comparisons for Mean_Performance_Rate:")
print(mpr_posthoc)
