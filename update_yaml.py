from ruamel.yaml import YAML, SafeDumper

class NLDumper(SafeDumper):
    def write_line_break(self, data=None):
        super().write_line_break(data)

        if len(self.indents) == 1:
            super().write_line_break()

def update_params(config_path_alias):
    # Modify original file with new alias values
    with open(config_path_alias, 'r') as f:
        params_raw = f.readlines()

    # Add _mod to the file name before .yaml
    config_path_alias_mod = config_path_alias.replace('.yaml', '_mod.yaml')
    with open(config_path_alias_mod, 'w') as f:
        f.writelines(params_raw)

    # Load the new YAML file using ruamel.yaml
    yaml = YAML(typ='safe', pure=True)
    with open(config_path_alias_mod, 'r') as f:
        params_alias = yaml.load(f)

    # Extract parameters that do not start with an alias ('/')
    params_no_alias = {k: v for k, v in params_alias.items() if k.startswith('/')}

    # Define the output configuration file path
    config_path = config_path_alias_mod.replace('_alias_mod', '')

    # Save the cleaned YAML file
    yaml = YAML(typ='unsafe', pure=True)
    with open(config_path, 'w') as f:
        yaml.dump(params_no_alias, f)

    return config_path

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        update_params(sys.argv[1])
    else:
        print("Usage: python update_yaml.py <config_path_alias>")
