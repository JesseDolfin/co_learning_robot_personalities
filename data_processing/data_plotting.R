# Load necessary libraries
library(dplyr)
library(ggplot2)
library(tidyr)
library(stringr)
library(assertthat) # For column existence checks
library(viridis)    # For a nicer color palette

#' Plot summary metrics and personality identification on a polar plot.
#'
#' @param data_collection_dir Directory where the CSV summary file is located.
#' @param summary_csv_filename Name of the summary CSV file.
#' @param circle Logical. If TRUE, restrict personality points to lie on a unit circle.
#' 
#' This function:
#' 1. Reads a summary CSV file with participant results.
#' 2. Filters out baseline personality data.
#' 3. Creates and saves:
#'    - A bar chart with mean performance rates (MPR) by personality.
#'    - A bar chart with average cumulative rewards by personality.
#'    - A Cartesian scatter plot (optionally restricted to a circle) of personality traits 
#'      (patient-impatient vs leader-follower).
plot_summary_metrics <- function(data_collection_dir, 
                                 summary_csv_filename,
                                 circle = TRUE) {
  
  # Construct the full path to the CSV file
  summary_csv_path <- file.path(data_collection_dir, summary_csv_filename)
  
  # Check if the CSV file exists
  if (!file.exists(summary_csv_path)) {
    stop("The summary CSV file does not exist at path: ", summary_csv_path)
  }
  
  # Load the results
  final_results <- read.csv(summary_csv_path)
  
  # Check necessary columns
  required_cols <- c("Participant", "Personality", "Mean_Performance_Rate", 
                     "Cumulative_Reward", "Patient_Impatient_Score", "Leader_Follower_Score")
  
  missing_cols <- setdiff(required_cols, colnames(final_results))
  if (length(missing_cols) > 0) {
    warning("The following required columns are missing: ", paste(missing_cols, collapse=", "))
  }
  
  # Filter out baseline personality if present
  final_results <- final_results %>%
    filter(!Personality %in% c("personality_type_baseline", "baseline"))
  
  # Convert Participant and Personality to factors for consistent plotting
  final_results <- final_results %>%
    mutate(
      Participant = as.factor(Participant),
      Personality = as.factor(Personality)
    )
  
  # ---------------------------------------------------------------
  # Helper functions for summarizing and plotting
  # ---------------------------------------------------------------
  
  summarize_with_ci <- function(data, measure_col, group_col = "Personality") {
    # Summarize data by group_col, computing mean, sd, and 95% CI
    data %>%
      group_by(!!sym(group_col)) %>%
      summarise(
        Mean = mean(.data[[measure_col]], na.rm = TRUE),
        SD = sd(.data[[measure_col]], na.rm = TRUE),
        N = sum(!is.na(.data[[measure_col]])),
        CI = ifelse(N > 1, qt(0.975, df = N - 1) * (SD / sqrt(N)), NA_real_),
        .groups = "drop"
      )
  }
  
  create_bar_plot <- function(df, x_col, y_col, fill_col, y_label, title, subtitle = NULL, y_limits = NULL, percent_scale = FALSE) {
    p <- ggplot(df, aes(x = .data[[x_col]], y = Mean, fill = .data[[fill_col]])) +
      geom_bar(stat = "identity", width = 0.7) +
      geom_errorbar(aes(ymin = Mean - SD, ymax = Mean + SD), width = 0.2) +
      theme_minimal(base_size = 14) +
      scale_fill_viridis_d(option = "D", end = 0.9) +
      labs(
        title = title,
        subtitle = subtitle,
        x = x_col,
        y = y_label
      ) +
      theme(
        axis.text.x = element_text(angle = 45, hjust = 1),
        legend.position = "none"
      )
    if (!is.null(y_limits)) {
      p <- p + scale_y_continuous(limits = y_limits, 
                                  labels = if (percent_scale) scales::percent_format(scale = 1) else waiver())
    } else if (percent_scale) {
      p <- p + scale_y_continuous(labels = scales::percent_format(scale = 1))
    }
    return(p)
  }
  
  # ---------------------------------------------------------------
  # MPR Plot
  # ---------------------------------------------------------------
  summary_mpr <- summarize_with_ci(final_results, "Mean_Performance_Rate", "Personality")
  mpr_plot <- create_bar_plot(
    df = summary_mpr,
    x_col = "Personality",
    y_col = "Mean",
    fill_col = "Personality",
    y_label = "Mean Performance Rate (%)",
    title = "Mean Performance Rate by Personality",
    subtitle = "Error bars represent ±1 SD",
    y_limits = c(0, 100),
    percent_scale = TRUE
  )
  
  # Save the MPR plot
  mpr_plot_filename <- file.path(data_collection_dir, "Average_Mean_Performance_Rate.png")
  ggsave(filename = mpr_plot_filename, plot = mpr_plot, width = 8, height = 6)
  message("MPR plot saved to: ", mpr_plot_filename)
  
  # ---------------------------------------------------------------
  # Cumulative Reward Plot
  # ---------------------------------------------------------------
  summary_reward <- summarize_with_ci(final_results, "Cumulative_Reward", "Personality")
  cumulative_reward_plot <- create_bar_plot(
    df = summary_reward,
    x_col = "Personality",
    y_col = "Mean",
    fill_col = "Personality",
    y_label = "Cumulative Reward",
    title = "Average Cumulative Reward by Personality",
    subtitle = "Error bars represent ±1 SD"
  )
  
  # Save the Cumulative Reward plot
  cumulative_reward_plot_filename <- file.path(data_collection_dir, "Average_Cumulative_Reward.png")
  ggsave(filename = cumulative_reward_plot_filename, plot = cumulative_reward_plot, width = 8, height = 6)
  message("Cumulative Reward plot saved to: ", cumulative_reward_plot_filename)
  
  # ---------------------------------------------------------------
  # Polar Plot for Personality Scores
  # ---------------------------------------------------------------
  
  # Compute averages and normalize Patient-Impatient and Leader-Follower scores
  polar_plot_data <- final_results %>%
    group_by(Personality) %>%
    summarise(
      Avg_Patient_Impatient = mean(Patient_Impatient_Score, na.rm = TRUE),
      Avg_Leader_Follower = mean(Leader_Follower_Score, na.rm = TRUE),
      .groups = "drop"
    ) %>%
    mutate(
      # Normalize to [-1, 1] if the data range is assumed to be ±3 originally
      Avg_Patient_Impatient = Avg_Patient_Impatient / 3,
      Avg_Leader_Follower = Avg_Leader_Follower / 3
    )
  
  # If circle == TRUE, restrict to unit circle if radius > 1
  if (circle) {
    polar_plot_data <- polar_plot_data %>%
      rowwise() %>%
      mutate(
        radius = sqrt(Avg_Patient_Impatient^2 + Avg_Leader_Follower^2),
        Avg_Patient_Impatient = ifelse(radius > 1, Avg_Patient_Impatient / radius, Avg_Patient_Impatient),
        Avg_Leader_Follower = ifelse(radius > 1, Avg_Leader_Follower / radius, Avg_Leader_Follower)
      ) %>%
      ungroup() %>%
      select(-radius)
  }
  
  # Generate circle coordinates for reference lines
  circle_coords <- function(r=1, n=200) {
    tibble(
      x = r * cos(seq(0, 2*pi, length.out = n)),
      y = r * sin(seq(0, 2*pi, length.out = n))
    )
  }
  
  # Concentric circles at different radii
  circle1 <- circle_coords(1)
  circle2 <- circle_coords(0.75)
  circle3 <- circle_coords(0.5)
  circle4 <- circle_coords(0.25)
  
  polar_plot <- ggplot() +
    # Draw reference circles
    geom_path(data = circle1, aes(x = x, y = y), color = "black") +
    geom_path(data = circle2, aes(x = x, y = y), color = "black", linetype = "dotted") +
    geom_path(data = circle3, aes(x = x, y = y), color = "black", linetype = "dotted") +
    geom_path(data = circle4, aes(x = x, y = y), color = "black", linetype = "dotted") +
    # Add horizontal and vertical reference lines
    geom_hline(yintercept = 0, linetype = "dashed", color = "gray") +
    geom_vline(xintercept = 0, linetype = "dashed", color = "gray") +
    # Plot points
    geom_point(
      data = polar_plot_data, 
      aes(x = Avg_Patient_Impatient, y = Avg_Leader_Follower, color = Personality), 
      size = 4
    ) +
    scale_color_viridis_d(option = "D", end = 0.9) +
    coord_fixed(xlim = c(-1.1, 1.1), ylim = c(-1.1, 1.1)) +
    theme_minimal(base_size = 14) +
    labs(
      title = if (circle) "Perceived Personality Scores (Restricted to Circle)" else "Perceived Personality Scores",
      subtitle = "Patient (-1) to Impatient (1) vs. Follower (-1) to Leader (1)",
      x = "← Patient           Impatient → ",
      y = "← Follower           Leader → "
    ) +
    theme(
      legend.position = "right",
      axis.title.x = element_text(margin = margin(t = 10)),
      axis.title.y = element_text(margin = margin(r = 10))
    )
  
  # Save the polar (Cartesian) plot
  polar_plot_filename <- file.path(
    data_collection_dir, 
    if(circle) "Cartesian_Scatter_Plot_with_Circle.png" else "Cartesian_Scatter_Plot_noCircle.png"
  )
  ggsave(filename = polar_plot_filename, plot = polar_plot, width = 8, height = 8)
  message("Polar personality plot saved to: ", polar_plot_filename)
  
  # Optionally display the plots in console
  print(mpr_plot)
  print(cumulative_reward_plot)
  print(polar_plot)
}


data_collection_dir <- "/home/jesse/thesis/src/co_learning_robot_personalities/data_collection"
summary_csv_filename <- "summary_metrics.csv"
plot_summary_metrics(data_collection_dir, summary_csv_filename, circle = TRUE)
