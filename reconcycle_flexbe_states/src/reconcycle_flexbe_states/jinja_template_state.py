#!/usr/bin/python

from flexbe_core import EventState, Logger

class {{state_name}}State(EventState):

    def __init__(self, {%for param in state_params %}{{param}}{% if not loop.last %}, {% endif %}{% endfor %}):
        super({{state_name}}State, self).__init__(outcomes = ['continue', 'failed'])
        {% for param in state_params -%}
        self.{{param}} = {{param}}
        {%- endfor %}
        
    def on_enter(self, userdata):
        {% set jinja_param_list = [] -%}
        {%for method_param in method_params -%}
        {%if method_param not in state_params -%}
        self.{{method_param}} = userdata.{{method_param}}
        {% endif %}
        {%- endfor %}
        userdata.{{userdata_skill_name}}.{{method_name}}(
            {%for method_param in method_params %}self.{{method_param}}{% if not loop.last %},{% endif %}{% endfor %})

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        pass