# -------------------------------------------------------------------------
# MANOVA_All_Metrics.R
#
# Purpose:
# Perform a repeated-measures MANOVA on all dependent variables (metrics)
# across different personalities, with Participant as a random factor.
#
# Requirements:
# - Ensure that the dataset contains the metrics and columns:
#   Participant, Personality, and numeric metrics for MANOVA.
# -------------------------------------------------------------------------

# Load necessary libraries
library(dplyr)
library(car)  # For MANOVA and multivariate tests
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

# Check required columns
required_cols <- c("Participant", "Personality")
missing_cols <- setdiff(required_cols, colnames(data))
if (length(missing_cols) > 0) {
  stop("The following required columns are missing: ", paste(missing_cols, collapse = ", "))
}

# Identify dependent variables (numeric metrics)
dependent_vars <- setdiff(colnames(data), c("Participant", "Personality"))

# Convert Personality and Participant to factors
data <- data %>%
  mutate(Personality = as.factor(Personality),
         Participant = as.factor(Participant))

# Debug: Check a sample of the data and structure
message("Preview of the data:")
print(head(data))
str(data)

# Prepare data for MANOVA
# Reshape data into long format for multivariate analysis
manova_data <- reshape(
  data, 
  varying = dependent_vars,
  v.names = "value",
  timevar = "measure",
  times = dependent_vars,
  idvar = c("Participant", "Personality"),
  direction = "long"
)

# Fit MANOVA model
manova_model <- lm(value ~ Personality * measure + Participant, data = manova_data)

# Perform MANOVA using the `car` package
manova_results <- Anova(
  manova_model, 
  idata = data.frame(measure = factor(dependent_vars)),
  idesign = ~measure,
  type = "III"
)

# Print MANOVA results
message("MANOVA summary:")
print(summary(manova_results))

# Univariate ANOVA for each dependent variable
message("Univariate ANOVA for each metric:")
for (var in dependent_vars) {
  message("Analyzing variable: ", var)
  
  # Perform ANOVA for each metric
  anova_model <- aov(as.formula(paste(var, "~ Personality + Error(Participant/Personality)")), data = data)
  
  # Print ANOVA summary
  print(summary(anova_model))
}

# Post-hoc tests for each dependent variable
message("Post-hoc tests for each metric:")
for (var in dependent_vars) {
  message("Post-hoc tests for ", var, ":")
  
  # Fit the model without the Error() term for post-hoc compatibility
  posthoc_model <- aov(as.formula(paste(var, "~ Personality")), data = data)
  
  # Perform pairwise comparisons using emmeans
  posthoc_results <- emmeans(posthoc_model, pairwise ~ Personality)
  
  # Print results
  print(posthoc_results)
}
