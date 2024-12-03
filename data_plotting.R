# Load necessary libraries
library(dplyr)
library(ggplot2)

# Function to plot and save Mean Performance Rate and Cumulative Reward
plot_summary_metrics <- function(data_collection_dir, summary_csv_filename) {
  # Construct the full path to the CSV file
  summary_csv_path <- file.path(data_collection_dir, summary_csv_filename)
  
  # Check if the CSV file exists
  if (!file.exists(summary_csv_path)) {
    stop(paste("The summary CSV file does not exist at path:", summary_csv_path))
  }
  
  # Load the results
  final_results <- read.csv(summary_csv_path)
  
  # Ensure the necessary columns exist
  required_columns <- c("Participant", "Personality", "Mean_Performance_Rate", "Cumulative_Reward")
  missing_columns <- setdiff(required_columns, names(final_results))
  if (length(missing_columns) > 0) {
    stop(paste("The following required columns are missing from the CSV file:", paste(missing_columns, collapse = ", ")))
  }
  
  # Convert Participant and Personality to factors for consistent plotting
  final_results$Participant <- as.factor(final_results$Participant)
  final_results$Personality <- as.factor(final_results$Personality)
  
  # Create Mean Performance Rate (MPR) plot
  mpr_plot <- ggplot(final_results, aes(x = Personality, y = Mean_Performance_Rate, fill = Personality)) +
    geom_bar(stat = "identity", position = "dodge") +
    facet_wrap(~ Participant) +  # Separate plots for each participant
    theme_minimal() +
    labs(
      title = "Mean Performance Rate (MPR) by Personality",
      x = "Personality",
      y = "Mean Performance Rate (%)"
    ) +
    theme(axis.text.x = element_text(angle = 45, hjust = 1))
  
  # Save the MPR plot to the data_collection directory
  mpr_plot_filename <- file.path(data_collection_dir, "Mean_Performance_Rate_by_Personality.png")
  ggsave(filename = mpr_plot_filename, plot = mpr_plot, width = 10, height = 6)
  message(paste("MPR plot saved to:", mpr_plot_filename))
  
  # Create Cumulative Reward plot
  cumulative_reward_plot <- ggplot(final_results, aes(x = Personality, y = Cumulative_Reward, fill = Personality)) +
    geom_bar(stat = "identity", position = "dodge") +
    facet_wrap(~ Participant) +  # Separate plots for each participant
    theme_minimal() +
    labs(
      title = "Cumulative Reward by Personality",
      x = "Personality",
      y = "Cumulative Reward"
    ) +
    theme(axis.text.x = element_text(angle = 45, hjust = 1))
  
  # Save the Cumulative Reward plot to the data_collection directory
  cumulative_reward_plot_filename <- file.path(data_collection_dir, "Cumulative_Reward_by_Personality.png")
  ggsave(filename = cumulative_reward_plot_filename, plot = cumulative_reward_plot, width = 10, height = 6)
  message(paste("Cumulative Reward plot saved to:", cumulative_reward_plot_filename))
  
  # Optionally, display the plots
  print(mpr_plot)
  print(cumulative_reward_plot)
}

# Example usage:
# Define the data collection directory and CSV filename
data_collection_dir <- "/home/jesse/thesis/src/co_learning_robot_personalities/data_collection"  # Adjust this path if needed
summary_csv_filename <- "summary_metrics_all_participants.csv"

# Call the function to generate and save the plots
plot_summary_metrics(data_collection_dir, summary_csv_filename)
