# Load necessary libraries
library(dplyr)
library(ggplot2)
library(tidyr)
library(stringr)
library(assertthat) # For column existence checks
library(viridis)    # For a nicer color palette
library(GGally)     # For correlation matrix plot

#' Plot summary metrics and personality identification on a polar plot,
#' plus additional scatterplots linking perceived personality to co-learning metrics.
#'
#' @param data_collection_dir Directory where the CSV summary file is located.
#' @param summary_csv_filename Name of the summary CSV file.
#' @param circle Logical. If TRUE, restrict personality points to lie on a unit circle (in the polar plot).
#' 
plot_qgap_trends <- function(per_episode_file, output_dir) {
  require(ggplot2)
  require(dplyr)
  require(stringr)
  
  # Read and prepare data
  df <- read.csv(per_episode_file) %>%
    mutate(
      Personality = str_remove(Personality, "personality_type_"),
      Personality = factor(Personality, 
                           levels = c("follower", "patient", "leader", "impatient"))
    ) %>%
    filter(!is.na(AvgQGap)) %>%
    group_by(Personality, Episode) %>%
    summarise(
      Mean_QGap = mean(AvgQGap, na.rm = TRUE),
      SE_QGap = sd(AvgQGap, na.rm = TRUE) / sqrt(n()),
      .groups = "drop"
    )
  
  # Create plot
  p <- ggplot(df, aes(x = Episode, y = Mean_QGap, color = Personality)) +
    geom_line(linewidth = 1.2) +
    geom_point(size = 3) +
    geom_ribbon(aes(ymin = Mean_QGap - 1.96*SE_QGap,
                    ymax = Mean_QGap + 1.96*SE_QGap,
                    fill = Personality),
                alpha = 0.0, colour = NA) +
    scale_color_viridis_d(end = 0.9) +
    scale_fill_viridis_d(end = 0.9) +
    labs(
      title = "Average Q-Gap Over Learning Episodes by Personality",
      x = "Episode Number",
      y = "Average Q-Gap",
    ) +
    theme_minimal(base_size = 14) +
    theme(
      legend.position = "right",
      panel.grid.minor = element_blank(),
      plot.caption = element_text(hjust = 0, face = "italic"),
      axis.text.x = element_text(angle = 45, hjust = 1)
    )
  
  # Save plot
  output_file <- file.path(output_dir, "Qgap_Over_Episodes.png")
  ggsave(output_file, p, width = 10, height = 6, dpi = 300)
  message("Q-Gap plot saved to: ", output_file)
  
  return(p)
}
plot_entropy_over_episodes <- function(data_collection_dir) {
  require(ggplot2)
  require(dplyr)
  require(stringr)
  
  # Read per-episode data
  per_episode_file <- file.path(data_collection_dir, "per_episode_metrics.csv")
  if (!file.exists(per_episode_file)) {
    stop("Per-episode metrics file not found: ", per_episode_file)
  }
  
  # Process data
  entropy_data <- read.csv(per_episode_file) %>%
    filter(!str_detect(Personality, "baseline")) %>%
    mutate(Personality = str_remove(Personality, "personality_type_")) %>%
    group_by(Personality, Episode) %>%
    summarise(
      Mean_Entropy = mean(Entropy, na.rm = TRUE),
      SE_Entropy = sd(Entropy, na.rm = TRUE) / sqrt(n()),
      .groups = "drop"
    ) %>%
    mutate(Personality = factor(Personality,
                                levels = c("follower", "patient", "leader", "impatient")))
  
  # Create plot (WITH POINTS ADDED)
  p <- ggplot(entropy_data, aes(x = Episode, y = Mean_Entropy, 
                                color = Personality, fill = Personality)) +
    geom_line(linewidth = 1.2) +
    geom_point(size = 3) +  # <--- THIS LINE ADDED
    geom_ribbon(aes(ymin = Mean_Entropy - 1.96*SE_Entropy,
                    ymax = Mean_Entropy + 1.96*SE_Entropy),
                alpha = 0.0, colour = NA) +
    scale_color_viridis_d(end = 0.9) +
    scale_fill_viridis_d(end = 0.9) +
    labs(
      title = "Average Entropy Over Learning Episodes by Personality",
      x = "Episode Number",
      y = "Average Entropy"
    ) +
    theme_minimal(base_size = 14) +
    theme(
      legend.position = "right",
      panel.grid.minor = element_blank(),
      axis.text.x = element_text(angle = 45, hjust = 1)
    )
  
  # Save plot
  output_file <- file.path(data_collection_dir, "Entropy_Over_Episodes.png")
  ggsave(output_file, p, width = 10, height = 6, dpi = 300)
  message("Entropy trend plot saved to: ", output_file)
  
  return(p)
}
plot_summary_metrics <- function(data_collection_dir, 
                                 summary_csv_filename,
                                 circle = TRUE) {
  
  # ------------------------------------------------------------------------
  # 0) Load data
  # ------------------------------------------------------------------------
  summary_csv_path <- file.path(data_collection_dir, summary_csv_filename)
  
  if (!file.exists(summary_csv_path)) {
    stop("The summary CSV file does not exist at path: ", summary_csv_path)
  }
  
  final_results <- read.csv(summary_csv_path)
  
  # Check for necessary columns; if some are missing, we just warn
  required_cols <- c(
    "Participant", 
    "Personality", 
    "Mean_Performance_Rate",
    "Cumulative_Reward", 
    "Patient_Impatient_Score", 
    "Leader_Follower_Score",
    "Fluency_Score",
    "Total_Strategy_Changes",
    "Stability"
  )
  
  missing_cols <- setdiff(required_cols, colnames(final_results))
  if (length(missing_cols) > 0) {
    warning("The following required columns are missing: ", 
            paste(missing_cols, collapse = ", "))
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
  
  # ------------------------------------------------------------------------
  # Helper Functions
  # ------------------------------------------------------------------------
  
  summarize_with_ci <- function(data, measure_col, group_col = "Personality") {
    # Summarize data by group_col, computing mean, sd, and 95% CI
    data %>%
      group_by(!!sym(group_col)) %>%
      summarise(
        Mean = mean(.data[[measure_col]], na.rm = TRUE),
        SD   = sd(.data[[measure_col]], na.rm = TRUE),
        N    = sum(!is.na(.data[[measure_col]])),
        CI   = ifelse(N > 1, qt(0.975, df = N - 1) * (SD / sqrt(N)), NA_real_),
        .groups = "drop"
      )
  }
  
  create_bar_plot <- function(df, x_col, y_col, fill_col, y_label, title,
                              subtitle = NULL, y_limits = NULL, percent_scale = FALSE) {
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
      p <- p + coord_cartesian(ylim = y_limits)
    }
    if (percent_scale) {
      p <- p + scale_y_continuous(labels = scales::percent_format(scale = 1))
    }
    return(p)
  }
  
  # NEW HELPER: Create a scatterplot with a regression line + correlation
  create_scatter_with_corr <- function(df, x_col, y_col, color_col,
                                       x_label, y_label, plot_title,
                                       out_filename) {
    # Only consider rows that are not NA for both x_col, y_col
    df_clean <- df %>%
      filter(!is.na(.data[[x_col]]), !is.na(.data[[y_col]]))
    
    if (nrow(df_clean) < 2) {
      warning("Not enough data to plot scatter for ", x_col, " vs. ", y_col)
      return(NULL)
    }
    
    # Calculate correlation (Pearson)
    cor_val <- cor(df_clean[[x_col]], df_clean[[y_col]], method = "pearson")
    
    # Build plot
    p <- ggplot(df_clean, aes(x = .data[[x_col]], y = .data[[y_col]],
                              color = .data[[color_col]])) +
      geom_point(size = 3, alpha = 0.7) +
      geom_smooth(method = "lm", se = FALSE, color = "black", linetype = "dashed") +
      scale_color_viridis_d(end = 0.9) +
      theme_minimal(base_size = 14) +
      labs(
        title = paste0(plot_title, " (r = ", round(cor_val, 2), ")"),
        x = x_label,
        y = y_label,
        color = "Personality"
      ) +
      theme(legend.position = "right")
    
    # Save plot
    ggsave(filename = out_filename, plot = p, width = 8, height = 6)
    message("Scatter plot saved to: ", out_filename, 
            " [Correlation ~ ", round(cor_val, 2), "]")
    return(p)
  }
  
  # ------------------------------------------------------------------------
  # 1) Mean Performance Rate Plot
  # ------------------------------------------------------------------------
  if (all(c("Mean_Performance_Rate", "Personality") %in% colnames(final_results))) {
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
    
    mpr_plot_filename <- file.path(data_collection_dir, "Average_Mean_Performance_Rate.png")
    ggsave(filename = mpr_plot_filename, plot = mpr_plot, width = 8, height = 6)
    message("MPR plot saved to: ", mpr_plot_filename)
  }
  
  # ------------------------------------------------------------------------
  # 2) Cumulative Reward Plot
  # ------------------------------------------------------------------------
  if (all(c("Cumulative_Reward", "Personality") %in% colnames(final_results))) {
    final_results <- final_results %>%
      mutate(
        Reward_Type = ifelse(Cumulative_Reward < 0, "Negative", "Positive")
      )
    
    summary_reward <- final_results %>%
      group_by(Personality, Reward_Type) %>%
      summarise(
        Mean = mean(Cumulative_Reward, na.rm = TRUE),
        SD   = sd(Cumulative_Reward, na.rm = TRUE),
        N    = sum(!is.na(Cumulative_Reward)),
        CI   = ifelse(N > 1, qt(0.975, df = N - 1) * (SD / sqrt(N)), NA_real_),
        .groups = "drop"
      )
    
    cumulative_reward_plot <- ggplot(summary_reward, 
                                     aes(x = Personality, 
                                         y = Mean, 
                                         fill = Reward_Type)) +
      geom_bar(stat = "identity", 
               position = position_dodge(width = 0.8), 
               width = 0.7) +
      geom_errorbar(aes(ymin = Mean - SD, ymax = Mean + SD),
                    position = position_dodge(width = 0.8),
                    width = 0.2) +
      scale_fill_manual(values = c("Negative" = "red", "Positive" = "green")) +
      theme_minimal(base_size = 14) +
      labs(
        title = "Average Cumulative Reward by Personality",
        subtitle = "Grouped by Negative vs. Positive Rewards",
        x = "Personality",
        y = "Cumulative Reward"
      ) +
      theme(
        axis.text.x = element_text(angle = 45, hjust = 1),
        legend.position = "right"
      )
    
    cumulative_reward_plot_filename <- file.path(data_collection_dir, "Average_Cumulative_Reward_PosNeg.png")
    ggsave(filename = cumulative_reward_plot_filename, plot = cumulative_reward_plot, width = 8, height = 6)
    message("Cumulative Reward (Neg vs. Pos) plot saved to: ", cumulative_reward_plot_filename)
  }
  
  # ------------------------------------------------------------------------
  # 3) Fluency Plot
  # ------------------------------------------------------------------------
  if (all(c("Fluency_Score", "Personality") %in% colnames(final_results))) {
    summary_fluency <- summarize_with_ci(final_results, "Fluency_Score", "Personality")
    fluency_plot <- create_bar_plot(
      df = summary_fluency,
      x_col = "Personality",
      y_col = "Mean",
      fill_col = "Personality",
      y_label = "Average Fluency Score",
      title = "Average Fluency Score by Personality",
      subtitle = "Error bars represent ±1 SD"
    )
    
    fluency_plot_filename <- file.path(data_collection_dir, "Average_Fluency_Score.png")
    ggsave(filename = fluency_plot_filename, plot = fluency_plot, width = 8, height = 6)
    message("Fluency plot saved to: ", fluency_plot_filename)
  }
  
  # ------------------------------------------------------------------------
  # 4) Polar Plot of Perceived Personality
  # ------------------------------------------------------------------------
  if (all(c("Patient_Impatient_Score", "Leader_Follower_Score") %in% colnames(final_results))) {
    polar_plot_data <- final_results %>%
      group_by(Personality) %>%
      summarise(
        Avg_Patient_Impatient = mean(Patient_Impatient_Score, na.rm = TRUE),
        Avg_Leader_Follower   = mean(Leader_Follower_Score, na.rm = TRUE),
        .groups = "drop"
      ) %>%
      mutate(
        Avg_Patient_Impatient = Avg_Patient_Impatient / 3,
        Avg_Leader_Follower   = Avg_Leader_Follower / 3
      )
    
    if (circle) {
      polar_plot_data <- polar_plot_data %>%
        rowwise() %>%
        mutate(
          radius = sqrt(Avg_Patient_Impatient^2 + Avg_Leader_Follower^2),
          Avg_Patient_Impatient = ifelse(radius > 1, Avg_Patient_Impatient / radius, Avg_Patient_Impatient),
          Avg_Leader_Follower   = ifelse(radius > 1, Avg_Leader_Follower / radius, Avg_Leader_Follower)
        ) %>%
        ungroup() %>%
        select(-radius)
    }
    
    circle_coords <- function(r = 1, n = 200) {
      tibble(
        x = r * cos(seq(0, 2*pi, length.out = n)),
        y = r * sin(seq(0, 2*pi, length.out = n))
      )
    }
    circle1 <- circle_coords(1)
    circle2 <- circle_coords(0.75)
    circle3 <- circle_coords(0.5)
    circle4 <- circle_coords(0.25)
    
    polar_plot <- ggplot() +
      geom_path(data = circle1, aes(x = x, y = y), color = "black") +
      geom_path(data = circle2, aes(x = x, y = y), color = "black", linetype = "dotted") +
      geom_path(data = circle3, aes(x = x, y = y), color = "black", linetype = "dotted") +
      geom_path(data = circle4, aes(x = x, y = y), color = "black", linetype = "dotted") +
      geom_hline(yintercept = 0, linetype = "dashed", color = "gray") +
      geom_vline(xintercept = 0, linetype = "dashed", color = "gray") +
      geom_point(
        data = polar_plot_data, 
        aes(x = Avg_Patient_Impatient, y = Avg_Leader_Follower, color = Personality), 
        size = 6 # Increased dot size
      ) +
      scale_color_viridis_d(
        option = "D", 
        end = 0.9, 
        labels = c("Follower", "Impatient", "Leader", "Patient") # Replace with your desired abbreviations
      ) +
      coord_fixed(xlim = c(-1.1, 1.1), ylim = c(-1.1, 1.1)) +
      theme_minimal(base_size = 14) +
      labs(
        title = if (circle) "Perceived Personality Scores (Restricted to Circle)" else "Perceived Personality Scores",
        subtitle = "Patient (-1) to Impatient (1) vs. Follower (-1) to Leader (1)",
        x = "← Patient           Impatient → ",
        y = "← Follower           Leader → ",
        color = "Personality" # Legend title
      ) +
      theme(
        legend.position = "right",
        axis.title.x = element_text(margin = margin(t = 10)),
        axis.title.y = element_text(margin = margin(r = 10))
      )
    
    
    polar_plot_filename <- file.path(
      data_collection_dir, 
      if(circle) "Cartesian_Scatter_Plot_with_Circle.png" else "Cartesian_Scatter_Plot_noCircle.png"
    )
    ggsave(filename = polar_plot_filename, plot = polar_plot, width = 8, height = 8)
    message("Polar personality plot saved to: ", polar_plot_filename)
  }
  
  # ------------------------------------------------------------------------
  # 5) Bar Chart: Total Strategy Changes by Personality
  # ------------------------------------------------------------------------
  bar_plot_strategy_changes <- NULL
  if (all(c("Total_Strategy_Changes", "Personality") %in% colnames(final_results))) {
    summary_changes <- summarize_with_ci(final_results, "Total_Strategy_Changes", "Personality")
    bar_plot_strategy_changes <- create_bar_plot(
      df = summary_changes,
      x_col = "Personality",
      y_col = "Mean",
      fill_col = "Personality",
      y_label = "Mean Total Strategy Changes",
      title = "Mean Total Strategy Changes by Personality",
      subtitle = "Error bars represent ±1 SD"
    )
    
    sc_plot_filename <- file.path(data_collection_dir, "Mean_Total_Strategy_Changes.png")
    ggsave(filename = sc_plot_filename, plot = bar_plot_strategy_changes, width = 8, height = 6)
    message("Strategy Changes plot saved to: ", sc_plot_filename)
  }
  
  # ------------------------------------------------------------------------
  # 6) Scatter Plot: Stability vs. Total Strategy Changes
  # ------------------------------------------------------------------------
  scatter_stability_vs_changes <- NULL
  if (all(c("Stability", "Total_Strategy_Changes") %in% colnames(final_results))) {
    scatter_stability_vs_changes <- ggplot(final_results,
                                           aes(x = Total_Strategy_Changes, 
                                               y = Stability, 
                                               color = Personality)) +
      geom_point(size = 3, alpha = 0.7) +
      theme_minimal(base_size = 14) +
      scale_color_viridis_d(end = 0.9) +
      labs(
        title = "Stability vs. Total Strategy Changes",
        x = "Total Strategy Changes",
        y = "Stability (0 - 1)"
      ) +
      theme(legend.position = "right")
    
    stab_vs_ch_filename <- file.path(data_collection_dir, "Stability_vs_Strategy_Changes.png")
    ggsave(filename = stab_vs_ch_filename, plot = scatter_stability_vs_changes, width = 8, height = 6)
    message("Stability vs. Changes plot saved to: ", stab_vs_ch_filename)
  }
  
  # ------------------------------------------------------------------------
  # 7) Create the new scatter plots for personality vs. co-learning
  # ------------------------------------------------------------------------
  
  # 7a) Patient_Impatient_Score vs. Avg_QGap
  if (all(c("Patient_Impatient_Score", "Avg_QGap", "Personality") %in% colnames(final_results))) {
    out_file <- file.path(data_collection_dir, "Scatter_PatientImpatient_vs_QGap.png")
    scatter_pIS_qgap <- create_scatter_with_corr(
      df = final_results,
      x_col = "Patient_Impatient_Score",
      y_col = "Avg_QGap",
      color_col = "Personality",
      x_label = "Patient-Impatient Score",
      y_label = "Avg QGap (RL Confidence)",
      plot_title = "Patient vs. Impatient vs. QGap",
      out_filename = out_file
    )
  }
  
  # 7b) Patient_Impatient_Score vs. Avg_ActionConsistency
  if (all(c("Patient_Impatient_Score", "Avg_ActionConsistency", "Personality") %in% colnames(final_results))) {
    out_file <- file.path(data_collection_dir, "Scatter_PatientImpatient_vs_ActionConsistency.png")
    
    scatter_pIS_acons <- create_scatter_with_corr(
      df = final_results,
      x_col = "Patient_Impatient_Score",
      y_col = "Avg_ActionConsistency",
      color_col = "Personality",
      x_label = "Patient-Impatient Score",
      y_label = "Avg ActionConsistency",
      plot_title = "Patient vs. Impatient vs. ActionConsistency",
      out_filename = out_file
    )
    
    # If create_scatter_with_corr returns the ggplot object, we can do:
    if (!is.null(scatter_pIS_acons)) {
      scatter_pIS_acons <- scatter_pIS_acons +
        scale_color_viridis_d(
          name = "Personality",
          breaks = c("personality_type_follower",
                     "personality_type_impatient",
                     "personality_type_leader",
                     "personality_type_patient"),
          labels = c("Follower", "Impatient", "Leader", "Patient")
        )
      
      ggsave(filename = out_file, plot = scatter_pIS_acons, width = 8, height = 6)
    }
  }
  
  
  # 7c) Leader_Follower_Score vs. Avg_QGap
  if (all(c("Leader_Follower_Score", "Avg_QGap", "Personality") %in% colnames(final_results))) {
    out_file <- file.path(data_collection_dir, "Scatter_LeaderFollower_vs_QGap.png")
    scatter_lfs_qgap <- create_scatter_with_corr(
      df = final_results,
      x_col = "Leader_Follower_Score",
      y_col = "Avg_QGap",
      color_col = "Personality",
      x_label = "Leader-Follower Score",
      y_label = "Avg QGap (RL Confidence)",
      plot_title = "Leader vs. Follower vs. QGap",
      out_filename = out_file
    )
  }
  
  # ------------------------------------------------------------------------
  # 7) Scatter Plot: Mean Performance Rate vs. Cumulative Reward
  # ------------------------------------------------------------------------
  scatter_mpr_vs_reward <- NULL
  if (all(c("Mean_Performance_Rate", "Cumulative_Reward") %in% colnames(final_results))) {
    scatter_mpr_vs_reward <- ggplot(final_results,
                                    aes(x = Mean_Performance_Rate, 
                                        y = Cumulative_Reward, 
                                        color = Personality)) +
      geom_point(size = 3, alpha = 0.7) +
      theme_minimal(base_size = 14) +
      scale_color_viridis_d(end = 0.9) +
      labs(
        title = "Mean Performance Rate (%) vs. Cumulative Reward",
        x = "Mean Performance Rate (%)",
        y = "Cumulative Reward"
      ) +
      theme(legend.position = "right")
    
    mpr_vs_cr_filename <- file.path(data_collection_dir, "MPR_vs_Reward.png")
    ggsave(filename = mpr_vs_cr_filename, plot = scatter_mpr_vs_reward, width = 8, height = 6)
    message("MPR vs. Reward plot saved to: ", mpr_vs_cr_filename)
  }
  
  # ------------------------------------------------------------------------
  # 8) Box Plot: Stability by Personality
  # ------------------------------------------------------------------------
  box_stability <- NULL
  if (all(c("Stability", "Personality") %in% colnames(final_results))) {
    box_stability <- ggplot(final_results, 
                            aes(x = Personality, y = Stability, fill = Personality)) +
      geom_boxplot(alpha = 0.7, outlier.shape = 16, outlier.size = 2) +
      theme_minimal(base_size = 14) +
      scale_fill_viridis_d(end = 0.9, guide = "none") +  # no legend
      scale_x_discrete(
        labels = c("personality_type_follower"  = "Follower",
                   "personality_type_impatient" = "Impatient",
                   "personality_type_leader"    = "Leader",
                   "personality_type_patient"   = "Patient")
      ) +
      labs(
        title = "Stability Distribution by Personality",
        x = "Personality",
        y = "Stability (0 - 1)"
      ) +
      theme(
        axis.text.x = element_text(angle = 45, hjust = 1),
        legend.position = "none"
      )
    
    stability_box_filename <- file.path(data_collection_dir, "Stability_by_Personality.png")
    ggsave(filename = stability_box_filename, plot = box_stability, width = 8, height = 6)
    message("Stability box plot saved to: ", stability_box_filename)
  }
  
  
  # ------------------------------------------------------------------------
  # 9) Scatter Plot: Fluency Score vs. Mean Performance Rate
  # ------------------------------------------------------------------------
  scatter_fluency_vs_mpr <- NULL
  if (all(c("Fluency_Score", "Mean_Performance_Rate") %in% colnames(final_results))) {
    scatter_fluency_vs_mpr <- ggplot(final_results,
                                     aes(x = Fluency_Score, 
                                         y = Mean_Performance_Rate, 
                                         color = Personality)) +
      geom_point(size = 3, alpha = 0.7) +
      theme_minimal(base_size = 14) +
      scale_color_viridis_d(end = 0.9) +
      labs(
        title = "Fluency Score vs. Mean Performance Rate (%)",
        x = "Fluency Score",
        y = "Mean Performance Rate (%)"
      ) +
      theme(legend.position = "right")
    
    fluency_vs_mpr_filename <- file.path(data_collection_dir, "Fluency_vs_MPR.png")
    ggsave(filename = fluency_vs_mpr_filename, plot = scatter_fluency_vs_mpr, width = 8, height = 6)
    message("Fluency vs. MPR plot saved to: ", fluency_vs_mpr_filename)
  }
  
  # ------------------------------------------------------------------------
  # 10) Scatter Plot: Total Strategy Changes vs. Fluency Score 
  # ------------------------------------------------------------------------
  scatter_changes_vs_fluency <- NULL
  if (all(c("Total_Strategy_Changes", "Fluency_Score") %in% colnames(final_results))) {
    scatter_changes_vs_fluency <- ggplot(final_results,
                                         aes(x = Total_Strategy_Changes, 
                                             y = Fluency_Score, 
                                             color = Personality)) +
      geom_point(size = 3, alpha = 0.7) +
      theme_minimal(base_size = 14) +
      scale_color_viridis_d(end = 0.9) +
      labs(
        title = "Total Strategy Changes vs. Fluency Score",
        x = "Total Strategy Changes",
        y = "Fluency Score"
      ) +
      theme(legend.position = "right")
    
    changes_vs_fluency_filename <- file.path(data_collection_dir, "Strategy_Changes_vs_Fluency.png")
    ggsave(filename = changes_vs_fluency_filename, plot = scatter_changes_vs_fluency, width = 8, height = 6)
    message("Strategy Changes vs. Fluency plot saved to: ", changes_vs_fluency_filename)
  }
  
  # ------------------------------------------------------------------------
  # 11) Scatter Plot: Stability vs. Fluency Score 
  # ------------------------------------------------------------------------
  scatter_stability_vs_fluency <- NULL
  if (all(c("Stability", "Fluency_Score") %in% colnames(final_results))) {
    scatter_stability_vs_fluency <- ggplot(final_results,
                                           aes(x = Stability, 
                                               y = Fluency_Score, 
                                               color = Personality)) +
      geom_point(size = 3, alpha = 0.7) +
      theme_minimal(base_size = 14) +
      scale_color_viridis_d(end = 0.9) +
      labs(
        title = "Stability vs. Fluency Score",
        x = "Stability (0 - 1)",
        y = "Fluency Score"
      ) +
      theme(legend.position = "right")
    
    stability_vs_fluency_filename <- file.path(data_collection_dir, "Stability_vs_Fluency.png")
    ggsave(filename = stability_vs_fluency_filename, plot = scatter_stability_vs_fluency, width = 8, height = 6)
    message("Stability vs. Fluency plot saved to: ", stability_vs_fluency_filename)
  }
  
  # ------------------------------------------------------------------------
  # 12) Bar Charts for Q-table Metrics (Avg_Entropy, Avg_QGap, etc.)
  # ------------------------------------------------------------------------
  # Example: create bar charts for each metric if the column exists
  # Example: create bar charts for each metric if the column exists
  qtable_metrics <- c("Avg_Entropy", "Avg_QGap", "Avg_Convergence", "Avg_ActionConsistency")
  
  for (metric_col in qtable_metrics) {
    if (all(c(metric_col, "Personality") %in% colnames(final_results))) {
      
      # Summarize data by metric_col
      summary_metric <- summarize_with_ci(final_results, metric_col, "Personality")
      
      # Set y-axis limits only for Avg_ActionConsistency
      y_limits <- switch(
        metric_col,
        "Avg_ActionConsistency" = c(0.9, 1),
        "Avg_Entropy" = c(1.25, 2),
        NULL  # default
      )
      
  
      
      # Create bar plot
      metric_plot <- create_bar_plot(
        df = summary_metric,
        x_col = "Personality",
        y_col = "Mean",
        fill_col = "Personality",
        y_label = metric_col,
        title = paste("Average", metric_col, "by Personality"),
        subtitle = "Error bars represent ±1 SD",
        y_limits = y_limits
      )
      
      # Save the plot
      plot_filename <- file.path(data_collection_dir, paste0("Average_", metric_col, ".png"))
      ggsave(filename = plot_filename, plot = metric_plot, width = 8, height = 6)
      message(metric_col, " plot saved to: ", plot_filename)
    }
  }
  

  # ------------------------------------------------------------------------
  # X) Scatter Plot: Mean Performance Rate vs. Stability
  # ------------------------------------------------------------------------
  if (all(c("Mean_Performance_Rate", "Stability", "Personality") %in% colnames(final_results))) {
    out_file <- file.path(data_collection_dir, "Scatter_MPR_vs_Stability.png")
    
    scatter_mpr_stability <- create_scatter_with_corr(
      df           = final_results,
      x_col        = "Mean_Performance_Rate",
      y_col        = "Stability",
      color_col    = "Personality",
      x_label      = "Mean Performance Rate (%)",
      y_label      = "Stability (0 - 1)",
      plot_title   = "MPR vs. Stability",
      out_filename = out_file
    )
    
    # Optionally, print the plot to the console too:
    #print(scatter_mpr_stability)
  }
  
  
  
  # ------------------------------------------------------------------------
  # 13) Correlation Matrix for Key Numeric Columns
  # ------------------------------------------------------------------------
  
  # Here, we add Q-table metrics to numeric_cols if they exist:
  potential_qtable_cols <- c("Avg_Entropy", "Avg_QGap", "Avg_Convergence", "Avg_ActionConsistency")
  numeric_cols <- c(
    "Mean_Performance_Rate", "Cumulative_Reward", "Total_Strategy_Changes", 
    "Stability", "Fluency_Score", "Patient_Impatient_Score", "Leader_Follower_Score",
    potential_qtable_cols
  )
  
  # Check if these columns are present
  numeric_cols_present <- intersect(numeric_cols, colnames(final_results))
  
  correlation_matrix_plot <- NULL
  if (length(numeric_cols_present) > 1) {
    # Build a numeric-only dataframe
    numeric_data <- final_results %>%
      select(all_of(numeric_cols_present))
    
    # Create a correlation matrix plot using GGally
    correlation_matrix_plot <- ggpairs(
      numeric_data,
      title = "Correlation Matrix of Key Metrics"
    )
    
    corr_matrix_filename <- file.path(data_collection_dir, "Correlation_Matrix.png")
    # ggsave can handle ggpairs objects, but might need a slight tweak in size
    ggsave(filename = corr_matrix_filename, plot = correlation_matrix_plot, width = 20, height = 20)
    message("Correlation matrix plot saved to: ", corr_matrix_filename)
  }
  
  # ------------------------------------------------------------------------
  # 14) Entropy Over Episodes Plot
  # ------------------------------------------------------------------------
  message("\nGenerating entropy-over-episodes plot...")
  plot_entropy_over_episodes(data_collection_dir)
  
  # ------------------------------------------------------------------------
  # 14) q-gap Over Episodes Plot
  # -----------------------------------------------------------------
  plot_qgap_trends(
    per_episode_file = file.path(data_collection_dir, "per_episode_metrics.csv"),
    output_dir = data_collection_dir
  )
  
  # ------------------------------------------------------------------------
  # Print or Show Plots in Console
  # ------------------------------------------------------------------------
  # (You can comment out prints if you only want to save to files)
  if (exists("mpr_plot"))                     print(mpr_plot)
  if (exists("cumulative_reward_plot"))       print(cumulative_reward_plot)
  if (exists("fluency_plot"))                 print(fluency_plot)
  if (exists("polar_plot"))                   print(polar_plot)
  if (!is.null(bar_plot_strategy_changes))    print(bar_plot_strategy_changes)
  if (!is.null(scatter_stability_vs_changes)) print(scatter_stability_vs_changes)
  if (!is.null(scatter_mpr_vs_reward))        print(scatter_mpr_vs_reward)
  if (!is.null(box_stability))                print(box_stability)
  if (!is.null(scatter_fluency_vs_mpr))       print(scatter_fluency_vs_mpr)
  if (!is.null(scatter_changes_vs_fluency))   print(scatter_changes_vs_fluency)
  if (!is.null(scatter_stability_vs_fluency)) print(scatter_stability_vs_fluency)
  if (exists("scatter_pIS_qgap"))            print(scatter_pIS_qgap)
  if (exists("scatter_pIS_acons"))           print(scatter_pIS_acons)
  if (exists("scatter_lfs_qgap"))            print(scatter_lfs_qgap)
  if (exists("scatter_mpr_stability"))        print(print(scatter_mpr_stability))
}

# Example usage:
data_collection_dir <- "/home/jesse/thesis/src/co_learning_robot_personalities/data_collection"
summary_csv_filename <- "summary_metrics.csv"
plot_summary_metrics(data_collection_dir, summary_csv_filename, circle = TRUE)
