# Load necessary libraries
library(dplyr)
library(emmeans)

# Load the aggregated data
data <- read.csv("/home/jesse/thesis/src/co_learning_robot_personalities/data_collection/summary_metrics_all_participants.csv")

# Convert Personality and Participant to factors
data <- data %>%
  mutate(Personality = as.factor(Personality),
         Participant = as.factor(Participant))

# Debug: Check the data structure
print(head(data))

# Check structure and summarize data
str(data)
summary(data)

# Perform ANOVA for Mean_Performance_Rate
anova_mpr <- aov(Mean_Performance_Rate ~ Personality + Error(Participant/Personality), data = data)
summary(anova_mpr)

# Post-hoc tests for Mean_Performance_Rate
mpr_posthoc <- emmeans(anova_mpr, pairwise ~ Personality)
print(mpr_posthoc)
