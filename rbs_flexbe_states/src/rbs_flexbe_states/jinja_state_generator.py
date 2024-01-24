from jinja2 import Environment, FileSystemLoader
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
    state_name = element[0]
    method_name = element[1]
    state_params = [string.replace(' ', '') for string in element[2].split(',')]
    state_params = [param for param in state_params if param != '']
    method_params = [string.replace(' ', '') for string in element[3].split(',')]
    method_params = [param for param in method_params if param != '']
    input_keys = [string.replace(' ', '') for string in element[4].split(',')]
    input_keys = [param for param in input_keys if param != '']
    output_keys = [string.replace(' ', '') for string in element[5].split(',')]
    output_keys = [param for param in output_keys if param != '']
    print(state_params)
    
    content = template.render(state_name=state_name, state_params=state_params, method_params=method_params, method_name=method_name, input_keys=input_keys, output_keys=output_keys)
    filename = f"{state_name.lower()}_state.py"
    with open(filename, mode="w", encoding="utf-8") as message:
        message.write(content)
        print(f"... wrote {filename}")