import requests
from dotenv import load_dotenv
import os

load_dotenv()
class EvergreenAgent():
    def __init__(self, modelName = "deepseek-chat", APIkey = os.getenv("API_KEY_Deepseek")):
        self.modelName = modelName
        self.URL = "https://api.deepseek.com/v1/chat/completions"
        self.chatHistory = []
        self.APIkey = APIkey
        self.initPrompt = """
You are an agent that processes job notes written by a tradesperson. Your task is to clean up and structure the notes into a clear, professional format.

Only use the information provided in the notes. Do not add extra commentary, assumptions, or customer conversation. Correct grammar, spelling, and punctuation. Present all tasks as clear bullet points under the correct headings.

---

🔧 Your output **must be valid HTML** using only the following elements:
- <div style="font-size: 10pt;"> to wrap each paragraph or section
- <strong> for section headings
- <ul> and <li> for bullet points
- <br> for inline line breaks
your output may have fewer than two subheading groups or more depending on the job notes given. in the below template, where it says "[REWRITE]" I instead want you to make it "[REWRITE] INCOMPLETE" if you determine the job is explicitly incomplete or requires more attention by the company after all relevant job notes have been read (there may be more than one).
If there is any indiciation that the job has been completed do not include the "F".
keep note of the date, if a note of the latest date indicates the job has been finished, mark it as finished not with the F tag.
---

🧱 Structure your output *exactly like this*:

<div style="font-size: 10pt;">
  [REWRITE]
</div>

<div style="font-size: 10pt;">
  Thank you for the opportunity to carry out the works at your premise.<br>
  Below is the scope of work carried out by the Evergreen team:
</div>
<br>
<div style="font-size: 10pt;">
  <strong>Worker/Date:</strong> [Insert name and date from notes here]
</div>

<div style="font-size: 10pt;">
  <strong>Work Completed:</strong>
  <ul>
    <li>Task Group 1 Subheading
      <ul>
        <li>Task 1</li>
        <li>Task 2</li>
      </ul>
    </li>
    <li>Task Group 2 Subheading
      <ul>
        <li>Task 1</li>
        <li>Task 2</li>
      </ul>
    </li>
  </ul>
</div>

<div style="font-size: 10pt;">
  Please consider Evergreen Electrical Services for your next electrical, solar or data job.<br>
  Regards, Evergreen Services.
</div>

---

📌 Ensure:
- `[REWRITE]` is included in the first <div>
- Each task is in its own `<li>` inside the `<ul>`
- No extra commentary or invented details
- if it looks like two seperate work notes have been given use the template above to ensure all work notes have their own sections
"""
        self.initPrompt2 = """
        the following text is an example of job notes that you should make the ones after this look like. Donot reply to this but every prompt after do reply even if they look similar to the following:
        thankyou for the opportunity to carry out the works at your premise.  
        below is the scope of work carried out by the evergreen team:

        Hayden Jenson, 22/08/25

        **Hot Water System Inspection**
        - Pump plug observed with excess water on top.
        - No cracks or penetrations in the connection box.
        - Silicon seal applied around the plug.

        **Air Conditioning System Inspection**
        - Unit is not old.
        - Isolator free of moisture.
        - Connection height is sufficient; no issues detected.

        please consider evergreen electrical services for your next electrical, solar or data jobs, regards evergreen services
        """

        self.chatHistory.append({"role": "user", "content": self.initPrompt})
        self.chatHistory.append({"role": "user", "content": self.initPrompt2})
        self.chatHistory.append({"role": "assistant", "content": "Understood. Ready to process job notes."})

    def sendNotes(self, jobNotes):
        self.chatHistory.append({"role": "user", "content": jobNotes})

        prompt_payload = {
            "model" : self.modelName,
            "stream" : False,
            "messages" : self.chatHistory
        }

        headers = {
        'Authorization': f'Bearer {self.APIkey}',
        'Content-Type': 'application/json'
        }

        
        print("⏳ Tidying up Job Notes...")
        response = requests.post(self.URL, headers=headers ,json=prompt_payload, timeout=300)

        self.chatHistory.pop(-1)

        if response.status_code == 200:
            data = response.json()
            assistant_message = data['choices'][0]['message']['content']
            return assistant_message
        else:
            print("❌ Error:", response.status_code, response.text)