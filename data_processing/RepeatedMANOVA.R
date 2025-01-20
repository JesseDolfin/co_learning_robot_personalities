###############################################################################
# File: MANOVA_and_RM_ANOVA_fixed.R
#
# Purpose:
#   1) Perform a repeated-measures MANOVA across multiple metrics (DVs).
#   2) If MANOVA shows an overall effect, follow up with:
#      - Repeated-measures ANOVAs for each metric.
#      - Post-hoc pairwise comparisons that respect within-subject design.
#
# Fixes:
#   - We properly construct `idata` with DV x Personality (44 rows),
#     and reorder the wide columns to match that order.
###############################################################################

# -----------------------------
# 0. Load Libraries
# -----------------------------
library(afex)         # for aov_ez() repeated-measures ANOVAs
library(emmeans)      # for post-hoc tests
library(tidyverse)    # for data wrangling
library(car)          # for MANOVA with repeated-measures design

# -----------------------------
# 1. Read and Prepare Data
# -----------------------------
data_file <- "/home/jesse/thesis/src/co_learning_robot_personalities/data_collection/summary_metrics.csv"
data <- read.csv(data_file, stringsAsFactors = FALSE)

# Convert relevant columns to factors
data$Participant <- factor(data$Participant)
data$Personality <- factor(data$Personality)

# Quick sanity checks
cat("Data preview:\n")
print(head(data))
cat("\nStructure:\n")
str(data)
cat("\nParticipant x Personality count:\n")
print(table(data$Participant, data$Personality))

# -----------------------------
# 2. Identify Dependent Variables
# -----------------------------
dependent_vars <- c(
  "Mean_Performance_Rate",
  "Cumulative_Reward",
  "Total_Strategy_Changes",
  "Stability",
  "Fluency_Score",
  "Patient_Impatient_Score",
  "Leader_Follower_Score",
  "Avg_Entropy",
  "Avg_QGap",
  "Avg_Convergence",
  "Avg_ActionConsistency"
)

# -----------------------------
# 3. Pivot Data to Wide Format
# -----------------------------
# We'll pivot each DV to wide format so each participant is a single row,
# and each (DV, personality-level) combination is a separate column.

personality_levels <- levels(data$Personality)  # e.g., 4 levels

pivot_dv_to_wide <- function(df, dv_col, subj_col = "Participant", within_col = "Personality") {
  df %>%
    select(all_of(c(subj_col, within_col, dv_col))) %>%
    pivot_wider(
      names_from  = all_of(within_col),
      values_from = all_of(dv_col)
    ) %>%
    rename_with(
      ~ paste0(dv_col, "_", .x),  # e.g., "Mean_Performance_Rate_follower"
      all_of(personality_levels)
    )
}

# Pivot each DV -> store in a list
wide_data_list <- lapply(dependent_vars, pivot_dv_to_wide, df = data)
# Merge all wide data frames on "Participant"
wide_data <- reduce(wide_data_list, left_join, by = "Participant")

cat("\n--- Wide Data Preview ---\n")
print(head(wide_data))

# -----------------------------
# 4. Construct idata for Repeated-Measures MANOVA
# -----------------------------
# We have 11 DVs x 4 Personality levels = 44 columns.
# We need an idata with exactly 44 rows, specifying which DV and which Personality
# each column corresponds to, in the *exact* order of our cbind(...).

dv_names <- dependent_vars
pers_levels <- personality_levels

idata <- expand.grid(
  DV          = factor(dv_names,      levels = dv_names),
  Personality = factor(pers_levels,   levels = pers_levels)
)
# This produces 11 x 4 = 44 rows.

# -----------------------------
# 5. Build cbind(...) in the same order as idata
# -----------------------------
# We'll iterate over each row of idata, pick the correct wide-data column,
# and bind them together in exactly that order.

all_columns <- vector("list", length = nrow(idata))  # 44 in total

for (i in seq_len(nrow(idata))) {
  this_dv   <- as.character(idata$DV[i])
  this_pers <- as.character(idata$Personality[i])
  
  # The matching column in wide_data:
  col_name <- paste0(this_dv, "_", this_pers)
  
  # Extract as a numeric vector
  all_columns[[i]] <- wide_data[[col_name]]
}

# cbind them to form the response matrix
resp_matrix <- do.call(cbind, all_columns)

cat("\nDimensions of resp_matrix:\n")
print(dim(resp_matrix))  # Should be 7 participants x 44 columns if you have 7 participants

# Double check each row is a participant
# We'll build a data frame for the model so we can pass it to lm()
manova_df <- data.frame(
  Participant = wide_data$Participant,
  resp_matrix
)

# We typically can do: lm(cbind(...) ~ 1), but let's do it with a matrix explicitly:
manova_model <- lm(
  formula = resp_matrix ~ 1
  # no data = ... needed here because we already have the matrix in the environment
)

# -----------------------------
# 6. Repeated-Measures MANOVA with car::Anova
# -----------------------------
cat("\n--- STEP 6: Running Repeated-Measures MANOVA ---\n")

manova_results <- Anova(
  manova_model,
  idata   = idata,
  idesign = ~ Personality,  # only personality is the repeated factor
  type    = "III"
)

cat("\nMANOVA Results (Multivariate Tests):\n")
print(summary(manova_results, multivariate = TRUE))

# If you want to see univariate results from the same model:
# cat("\nMANOVA Results (Univariate Tests):\n")
# print(summary(manova_results, univariate = TRUE))

# -----------------------------
# 7. Follow-Up: Repeated-Measures ANOVAs + Post-hoc for Each DV
# -----------------------------
cat("\n--- STEP 7: Univariate Repeated-Measures ANOVAs & Post-hoc ---\n")
for (dv in dependent_vars) {
  cat("\n=======================================\n")
  cat("DV:", dv, "\n")
  cat("=======================================\n")
  
  # Repeated-measures ANOVA with afex::aov_ez
  aov_result <- aov_ez(
    id   = "Participant",
    dv   = dv,
    data = data,
    within = "Personality",
    type = 3  # Type III SS
  )
  
  # Print the ANOVA table
  print(aov_result)
  
  # If you only want post-hoc if significant, you can check:
  # p_val <- aov_result$Anova$"Pr(>F)"[1]
  # if (p_val < 0.05) {
  #   # do post-hoc
  # }
  
  # For demonstration, always do post-hoc:
  posthoc <- emmeans(aov_result, pairwise ~ Personality, adjust = "tukey")
  cat("\nPost-hoc Pairwise Comparisons:\n")
  print(posthoc)
}

cat("\n\n*** Analysis Complete! ***\n")
