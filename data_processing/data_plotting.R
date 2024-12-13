# Load necessary libraries
library(dplyr)
library(ggplot2)
library(tidyr)  # Added tidyr for pivot_longer

# Function to plot and save summary metrics and personality identification on a polar plot
plot_summary_metrics <- function(data_collection_dir, summary_csv_filename) {
  # Construct the full path to the CSV file
  summary_csv_path <- file.path(data_collection_dir, summary_csv_filename)
  
  # Check if the CSV file exists
  if (!file.exists(summary_csv_path)) {
    stop(paste("The summary CSV file does not exist at path:", summary_csv_path))
  }
  
  # Load the results
  final_results <- read.csv(summary_csv_path)
  
  # Remove baseline personality
  final_results <- final_results %>%
    filter(Personality != "personality_type_baseline")
  
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
  
  # Compute and create a cartesian scatter plot with a circle overlay for Patient-Impatient and Leader-Follower scores
  # Compute averages and normalize
  polar_plot_data <- final_results %>%
    group_by(Personality) %>%
    summarise(
      Avg_Patient_Impatient = mean(Patient_Impatient_Score, na.rm = TRUE),
      Avg_Leader_Follower = mean(Leader_Follower_Score, na.rm = TRUE)
    ) %>%
    # Normalize to [-1, 1] if the data range is not already normalized
    mutate(
      Avg_Patient_Impatient = Avg_Patient_Impatient / 3,  # Divide by the max absolute value to normalize [-3,3] to [-1,1]
      Avg_Leader_Follower = Avg_Leader_Follower / 3
    )
  
  polar_plot_data <- polar_plot_data %>%
    mutate(
      # Calculate the radius
      radius = sqrt(Avg_Patient_Impatient^2 + Avg_Leader_Follower^2),
      # Scale values if radius > 1
      Avg_Patient_Impatient = ifelse(radius > 1, Avg_Patient_Impatient / radius, Avg_Patient_Impatient),
      Avg_Leader_Follower = ifelse(radius > 1, Avg_Leader_Follower / radius, Avg_Leader_Follower)
    ) %>%
    select(-radius)  # Remove the radius column if no longer needed
  
  
  
  # Create polar plot
  polar_plot <- ggplot(polar_plot_data, aes(x = Avg_Patient_Impatient, y = Avg_Leader_Follower, color = Personality)) +
    geom_point(size = 4) +
    geom_hline(yintercept = 0, linetype = "dashed", color = "gray") +
    geom_vline(xintercept = 0, linetype = "dashed", color = "gray") +
    annotate("path", x = cos(seq(0, 2 * pi, length.out = 100)), 
             y = sin(seq(0, 2 * pi, length.out = 100)), color = "black") +
    annotate("path", x = 0.75 * cos(seq(0, 2 * pi, length.out = 100)), 
             y = 0.75 * sin(seq(0, 2 * pi, length.out = 100)), color = "black", linetype = "dotted") +
    annotate("path", x = 0.5 * cos(seq(0, 2 * pi, length.out = 100)), 
             y = 0.5 * sin(seq(0, 2 * pi, length.out = 100)), color = "black", linetype = "dotted") +
    annotate("path", x = 0.25 * cos(seq(0, 2 * pi, length.out = 100)), 
             y = 0.25 * sin(seq(0, 2 * pi, length.out = 100)), color = "black", linetype = "dotted") +
    coord_fixed(xlim = c(-1, 1), ylim = c(-1, 1)) +
    theme_minimal() +
    labs(
      title = "Cartesian Scatter Plot with Circular Overlay",
      x = "Patient (-1) to Impatient (1)",
      y = "Follower (-1) to Leader (1)"
    ) +
    theme(axis.text = element_text(size = 12))
  
  # Save the updated polar plot
  polar_plot_filename <- file.path(data_collection_dir, "Cartesian_Scatter_Plot_with_Circle.png")
  ggsave(filename = polar_plot_filename, plot = polar_plot, width = 8, height = 6)
  message(paste("Updated Cartesian scatter plot with circular overlay saved to:", polar_plot_filename))
  
  # Optionally, display the plots
  print(mpr_plot)
  print(cumulative_reward_plot)
  print(polar_plot)
}

# Example usage:
# Define the data collection directory and CSV filename
data_collection_dir <- "/home/jesse/thesis/src/co_learning_robot_personalities/data_collection"  # Adjust this path if needed
summary_csv_filename <- "summary_metrics_all_participants.csv"

# Call the function to generate and save the plots
plot_summary_metrics(data_collection_dir, summary_csv_filename)
