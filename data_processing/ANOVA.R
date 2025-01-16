# -------------------------------------------------------------------------
# ANNOVA_All_Variables.R
#
# Purpose:
# Perform repeated-measures ANOVA for all variables in the dataset across 
# different personalities. Post-hoc comparisons are also conducted.
#
# Requirements:
# - The dataset "summary_metrics_all_participants.csv" should exist in
#   data_collection_dir.
# - The dataset should contain "Personality" and "Participant" columns,
#   along with numeric variables to analyze.
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
data_file <- file.path(data_collection_dir, "summary_metrics.csv")

# Check if the file exists
if (!file.exists(data_file)) {
  stop("Data file not found at: ", data_file)
}

# Load the aggregated data
data <- read.csv(data_file)

# Check that required columns exist
required_cols <- c("Personality", "Participant")
numeric_vars <- setdiff(colnames(data), required_cols)
missing_cols <- setdiff(required_cols, colnames(data))
if (length(missing_cols) > 0) {
  stop("The following required columns are missing: ", paste(missing_cols, collapse = ", "))
}

# Convert Personality and Participant to factors
data <- data %>%
  mutate(Personality = as.factor(Personality),
         Participant = as.factor(Participant))

# Debug: Check a sample of the data
message("Preview of the data:")
print(head(data))

# Loop through each numeric variable and perform repeated-measures ANOVA
for (var in numeric_vars) {
  message("Analyzing variable: ", var)
  
  # Perform repeated-measures ANOVA
  formula <- as.formula(paste(var, "~ Personality + Error(Participant/Personality)"))
  anova_result <- tryCatch({
    aov(formula, data = data)
  }, error = function(e) {
    message("Error analyzing ", var, ": ", e$message)
    NULL
  })
  
  # Print ANOVA summary if successful
  if (!is.null(anova_result)) {
    message("ANOVA summary for ", var, ":")
    print(summary(anova_result))
    
    # Post-hoc tests for variable using emmeans
    posthoc_result <- tryCatch({
      emmeans(anova_result, pairwise ~ Personality)
    }, error = function(e) {
      message("Error in post-hoc tests for ", var, ": ", e$message)
      NULL
    })
    
    if (!is.null(posthoc_result)) {
      message("Post-hoc comparisons for ", var, ":")
      print(posthoc_result)
    }
  }
}
