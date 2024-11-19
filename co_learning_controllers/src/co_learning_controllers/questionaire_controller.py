#!/usr/bin/env python3

import gspread
from google.oauth2.service_account import Credentials
import webbrowser
import pandas as pd
import os
import rospy

class GoogleFormHandler:
    def __init__(self, form_url, sheet_url, key_path):
        """
        Initializes the Google Form handler.

        :param form_url: URL of the Google Form
        :param sheet_url: URL of the Google Sheet linked to the form
        :param key_path: Path to the service account JSON key file
        """
        self.form_url = form_url
        self.sheet_url = sheet_url
        self.key_path = key_path

        # Authenticate with Google Sheets API
        scope = ["https://spreadsheets.google.com/feeds", "https://www.googleapis.com/auth/drive"]
        creds = Credentials.from_service_account_file(self.key_path, scopes=scope)
        self.client = gspread.authorize(creds)

    def open_google_form(self):
        """
        Opens the Google Form for the participant to fill out.
        """
        rospy.loginfo(f"Opening Google Form: {self.form_url}")
        webbrowser.open(self.form_url)
        input("Press Enter after completing the Google Form to continue...")

    def fetch_and_save_responses(self, dir):
        """
        Fetches the latest response from the Google Sheet and saves it.

        :param dir: Directory where responses should be saved
        """
        sheet = self.client.open_by_url(self.sheet_url).sheet1
        all_records = sheet.get_all_records()

        if not all_records:
            rospy.logwarn("No responses found in the Google Sheet.")
            return

        # Get the last record
        last_record = all_records[-1:]  # A list containing only the last record
        df = pd.DataFrame(last_record)

        save_path = os.path.join(dir, "form_responses.csv")
        os.makedirs(dir, exist_ok=True)

        df.to_csv(save_path, index=False)
        rospy.loginfo(f"Last form response saved to: {save_path}")

    def run_workflow(self, dir):
        """
        Executes the workflow:
        1. Opens the Google Form.
        2. Fetches and saves the latest response.

        :param personality_dir: Directory where responses should be saved
        """
        self.open_google_form()
        self.fetch_and_save_responses(dir=dir)


if __name__== "__main__":

    form_url = "https://forms.gle/xGV3pWaNoPVrXjHXA"
    sheet_url = "https://docs.google.com/spreadsheets/d/1iVvVxfakw5Un8Wk9xu2ObyB40vr6SW-ENc43ewN9g54/edit"
    key_path = "/home/jesse/thesis_cor_tud/Documentation/psyched-loader-422713-u4-efcf9b902f7b.json"
    personality_dir = "/home/jesse/thesis/src/co_learning_robot_personalities/data_collection/participant_-1/personality_type_fake"

    # Initialize the handler
    form_handler = GoogleFormHandler(
        form_url=form_url,
        sheet_url=sheet_url,
        key_path=key_path,
    )

    # Run the workflow
    print("Starting Google Form workflow...")
    form_handler.run_workflow(dir=personality_dir)
    print("Workflow completed.")