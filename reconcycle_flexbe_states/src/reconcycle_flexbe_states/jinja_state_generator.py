from jinja2 import Environment, FileSystemLoader
from robotblockset_python.panda_ros import panda_ros
from inspect import getmembers, isfunction
import csv

environment = Environment(loader=FileSystemLoader("."))
template = environment.get_template("jinja_template_state.py")

function_templates = []
CSV_STATE_TEMPLATE_FILENAME = 'jinja_states_template.csv'
with open(CSV_STATE_TEMPLATE_FILENAME, newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter = ';', quotechar = '|')
    for row in reader:
        function_templates.append(row)
        
function_templates = function_templates[1:] # Remove 1st row since it contains instructions

for element in function_templates:
    #state_name = 'Move'
    #method_name = 'CMove'
    ##state_params = ['controller']
    #method_params = ['target', 'time', 'controller']
    print(element)
    state_name = element[0]
    userdata_skill_name = element[1]
    method_name = element[2]
    state_params = element[3].split(',')
    method_params = element[4].split(',')
    
    content = template.render(state_name=state_name, userdata_skill_name = userdata_skill_name, state_params=state_params, method_params=method_params, method_name=method_name)
    filename = f"{state_name}State.py"
    with open(filename, mode="w", encoding="utf-8") as message:
        message.write(content)
        print(f"... wrote {filename}")