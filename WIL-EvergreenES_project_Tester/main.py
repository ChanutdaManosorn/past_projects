import requests
import json
import agent
import htmlUtility

from dotenv import load_dotenv
import os
load_dotenv()

ACCESS_TOKEN = os.getenv("API_KEY_Simpro")
BASE_URL = os.getenv("BASE_URL")
COMPANY_NAME = os.getenv("COMPANY_NAME")

headers = {
    'Authorization': f'Bearer {ACCESS_TOKEN}',
    'Accept': 'application/json',
    'Content-Type': 'application/json'
}

# check
# print("Simpro Key:", ACCESS_TOKEN[:6], "...")  # frist 6
# print("Base URL:", BASE_URL)
# print("Company:", COMPANY_NAME)

### API FUNCTIONS ###
def get_Jobs(ID): #gets list of all jobs 
    URL = BASE_URL+"companies/"+str(ID)+'/jobs/'
    return requests.get(URL, headers=headers).json()

def get_Company(): #finds evergreen electrical
    URL = BASE_URL+"companies/"
    companies = requests.get(URL, headers=headers).json()
    for company in companies:
        if company.get("Name") == COMPANY_NAME:
            idNum = company.get("ID")
            
            return requests.get(BASE_URL+"companies/"+str(idNum), headers=headers).json()    
    return "company not found!"

def get_Costomers(ID): #gets all customers
    URL = BASE_URL+"companies/"+str(ID)+'/customers/'
    return requests.get(URL, headers=headers).json()

def find_Customer(ID, GivenName, FamilyName): #finds a customer
    URL = BASE_URL+"companies/"+str(ID)+'/customers/'
    customers = requests.get(URL, headers=headers).json()
    for customer in customers:
        name = customer.get("GivenName").lower()
        surname = customer.get("FamilyName").lower()
        if name.strip() == GivenName.strip() and surname.strip() == FamilyName.strip():
            return customer
    return f"could not find customer with name: {GivenName} {FamilyName}"

def add_Customer(ID, Title, GivenName, FamilyName, Phone): #adds a customer
    customer = {
        "Title":Title,
        "GivenName":GivenName,
        "FamilyName":FamilyName,
        "Phone":Phone
    }
    URL = BASE_URL+"companies/"+str(ID)+'/customers/individuals/'
    requests.post(URL, headers=headers, data=json.dumps(customer))

def remove_Customer(ID, GivenName, FamilyName): #removes a customer
    URL = BASE_URL+"companies/"+str(ID)+'/customers/'
    customers = requests.get(URL, headers=headers).json()
    for customer in customers:
        if customer.get("GivenName") == GivenName and customer.get("FamilyName") == FamilyName:
            URL += "individuals/"+str(customer.get("ID"))
            requests.delete(URL, headers=headers)

def updateJob(ID, jobID, stageID):
    URL = BASE_URL+"companies/"+str(ID)+'/jobs/'+str(jobID)
    print(URL)
    payload = {"Stage": stageID}
    return requests.patch(URL, headers=headers, json=payload)


### TESTING ###
def test_Requests(ID): #function for running all test API requests and prints out resulting data
    company = get_Company()
    print(json.dumps(company, indent=2))

    ## gets all jobs from the company and prints
    print("getting all jobs...")
    jobList = get_Jobs(company.get("ID"))
    print(json.dumps(jobList, indent=2))

    ## get and display customers
    print("getting all customers...")
    customers = get_Costomers(company.get("ID"))
    print(json.dumps(customers, indent=2))

    print("adding customer...")
    add_Customer(company.get("ID"),"Mr","Mike", "Ross", "0422352436")
    print(json.dumps(get_Costomers(company.get("ID")), indent=2))

    print()

    print("removing customer...")
    remove_Customer(company.get("ID"), "Mike", "Ross")
    print(json.dumps(get_Costomers(company.get("ID")), indent=2))



    del company, jobList, customers
###


def get_Job_Logs(ID, JobID):
    URL = BASE_URL+"companies/"+str(ID)+"/jobs/"+str(JobID)+"/timelines/"
    response = requests.get(URL, headers=headers)
    res = response.json()
    valid_Notes = []
    for note in res:
        if note.get("Type") == "Work Order Technician Notes" or note.get("Type") == "Customer Note":
            valid_Notes.append(note)
    return valid_Notes  
class API:
    def __init__(self, headers):
        self.ID = get_Company().get("ID")
        self.headers = headers

    def updateJobs(self, evergreenAgent):
        jobData = get_Jobs(self.ID)
        completeJobs = []
        editedJobs = []
        for job in jobData:
            currentID = job.get("ID")
            URL = BASE_URL + 'companies/'+str(self.ID)+'/jobs/'+str(currentID)
            data = requests.get(URL, headers=headers).json()
            if (data.get("Stage") != "Complete"):
                continue

            content = htmlUtility.strip_html(job.get("Description"))
            if content.startswith("[REWRITE]"):
                print(f"skipping job...(#{currentID}) REASON: contains [REWRITE TAG]")
                continue
            result = get_Job_Logs(self.ID, job.get("ID"))
            if (not result):
                print(f"skipping job... (#{currentID}) REASON: no valid job notes found")
                continue
            text = ""
            for note in result:
                text += htmlUtility.strip_html(note.get("Message"))
                text += '\n'

            print(f"editing job #{currentID}...")

            editedMessage = evergreenAgent.sendNotes(text)

            payload = {
            "Description" : editedMessage
            }

            updateURL = BASE_URL+"companies/"+str(self.ID)+"/jobs/"+str(job.get("ID"))
            checker = requests.patch(updateURL, headers=headers, json=payload)

            if checker.status_code not in (200, 204):
                print("❌ Error updating job:")
                print(checker.text)
            else:
                if ("[REWRITE] INCOMPLETE" in editedMessage):
                    editedJobs.append(currentID)
                elif ("[REWRITE]" in editedMessage):
                    completeJobs.append(currentID)
                print(f"✅ Job #{currentID} updated successfully")

        print()
        print("Edited Jobs Marked Complete: ")
        for id in completeJobs:
            print(f"Job #{id}")

        print()
        print("Edited Jobs Deemed Incomplete: ")
        for id in editedJobs:
            print(f"Job #{id}")
        
        

running = True
testAPI = API(headers)
testAgent = agent.EvergreenAgent()
print("NEW RUN...")
while running:
    userInput = input("enter API query ('r' or 'q') ")
    if userInput == "q":
        running = False
        print("exiting program runtime...")
        break
    elif userInput == "r":
        testAPI.updateJobs(testAgent)