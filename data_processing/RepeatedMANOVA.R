# -------------------------------------------------------------------------
# RepeatedMANOVA.R
#
# Purpose:
# Perform a repeated-measures MANOVA on multiple dependent variables 
# across different personalities, with Participant as a random factor.
#
# Requirements:
# - The dataset "summary_metrics_all_participants.csv" should exist in
#   data_collection_dir.
# - The dataset should contain "Mean_Performance_Rate", "Cumulative_Reward",
#   "Personality", and "Participant" at minimum.
#
# Notes:
# - Ensure assumptions for MANOVA are met (multivariate normality, 
#   sphericity, etc.).
# - Consider transformations or non-parametric multivariate tests if 
#   assumptions are not met.
#
# Optional:
# - Add more dependent variables to the MANOVA if they exist in the dataset.
#   For example, if you have "Stability" or "Total_Strategy_Changes" that 
#   might be relevant, add them to the cbind() call.
# -------------------------------------------------------------------------

# Load necessary libraries
library(dplyr)
library(car)     # For multivariate analysis utilities
library(afex)    # For ANOVA-type analyses, optional if you prefer
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
required_cols <- c("Mean_Performance_Rate", "Cumulative_Reward", "Personality", "Participant")
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

# If additional dependent variables are available, add them to cbind():
# For example, if you have "Stability" and "Total_Strategy_Changes" you could do:
# cbind(Mean_Performance_Rate, Cumulative_Reward, Stability, Total_Strategy_Changes)
# Make sure these columns exist and are numeric.

# Run the repeated-measures MANOVA with at least two DVs:
manova_results <- manova(
  cbind(Mean_Performance_Rate, Cumulative_Reward) ~ Personality + 
    Error(Participant/Personality),
  data = data
)

# Print the MANOVA summary
message("MANOVA summary:")
print(summary(manova_results))

# Univariate ANOVAs for each dependent variable
message("Univariate ANOVAs for each dependent variable:")
print(summary.aov(manova_results))

# Post-hoc for Mean Performance Rate
# Using the same model structure as ANOVA:
mpr_model <- aov(Mean_Performance_Rate ~ Personality + Error(Participant/Personality), data = data)
mpr_posthoc <- emmeans(mpr_model, pairwise ~ Personality)
message("Post-hoc comparisons for Mean_Performance_Rate:")
print(mpr_posthoc)

# Post-hoc for Cumulative Reward
reward_model <- aov(Cumulative_Reward ~ Personality + Error(Participant/Personality), data = data)
reward_posthoc <- emmeans(reward_model, pairwise ~ Personality)
message("Post-hoc comparisons for Cumulative_Reward:")
print(reward_posthoc)

# If you added more dependent variables, consider running post-hoc tests for them as well.
