#!/usr/bin/python

from flexbe_core import EventState, Logger

class {{state_name}}State(EventState):

    def __init__(self, robot_name, {%for param in state_params %}{{param}}{% if not loop.last %}, {% endif %}{% endfor %}):
        super({{state_name}}State, self).__init__(
            outcomes = ['continue', 'failed'],
            input_keys = [{%for param in input_keys %}{{"'"+param+"'"}}{% endfor %}],
            {% if output_keys|length %} output_keys = [{%for param in output_keys %}{{"'"+param+"'"}}{% endfor %}]{% endif %})

        self.robot_name = robot_name
        {% if state_params|length > 0 %}
        {% for param in state_params %}
        self.{{param}} = {{param}}
        {%- endfor %}
        {% endif %}
        
    def on_enter(self, userdata):
        {% set jinja_param_list = [] -%}
        {%for method_param in method_params -%}
        {%if method_param not in state_params -%}
        self.{{method_param}} = userdata.{{method_param}}
        {% endif %}
        {%- endfor %}
        {% if method_params|length > 0 %}
        userdata.robots[self.robot_name].{{method_name}}(
            {% for method_param in method_params %}{{method_param}} = self.{{method_param}}{% if not loop.last %},{% endif %}{% endfor %})
        {% else %}
        userdata.robots[self.robot_name].{{method_name}}()
        {% endif %}

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        pass
