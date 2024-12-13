# Load necessary libraries
library(dplyr)
library(car)
library(afex)
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

# Run the repeated-measures MANOVA
manova_results <- manova(
  cbind(Mean_Performance_Rate, Cumulative_Reward) ~ Personality + 
    Error(Participant/Personality),
  data = data
)

# Print the MANOVA summary
summary(manova_results)

# Univariate ANOVAs for each dependent variable
summary.aov(manova_results)

# Post-hoc for Mean Performance Rate
mpr_posthoc <- emmeans(aov(Mean_Performance_Rate ~ Personality + Error(Participant/Personality), data = data),
                       pairwise ~ Personality)
print(mpr_posthoc)

# Post-hoc for Cumulative Reward
reward_posthoc <- emmeans(aov(Cumulative_Reward ~ Personality + Error(Participant/Personality), data = data),
                          pairwise ~ Personality)
print(reward_posthoc)
