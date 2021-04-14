#!/usr/bin/env python
#

"""
Utilities
"""

def string_to_type(s):
    """Convert string to str, int, float

    Args:
        s (str): string

    Returns:
        str, int or float
    """
    try:
        return int(s)
    except ValueError:
        try:
            return float(s)
        except ValueError:
            return s


def zip_dict(keys, values):
    """Zip keys and values lists into a dictionary

        {keys:[key], values:[value]} ==> {key:value}
        value: 'abc;10;3.14' ==> ['abc', 10, 3.14]
        value: 'abc' ==> 'abc'
        value: '3.14' ==> 3.14
    """
    key_value = {}

    for index, key in enumerate(keys):
        value = values[index].split(';')
        value = [string_to_type(v) for v in value]
        if len(value) == 1:
            value = value[0]
        key_value[key] = value

    return key_value


def unzip_dict(key_value_dict):
    """Unzip a dictionary into lists of keys and values

        {key:value} ==> {output_keys:[key], output_values:[value]}
        value: ['abc', 10, 3.14] ==> 'abc;10;3.14'
        value: 'abc' ==> 'abc'
        value: 3.14 ==> '3.14'
    """
    keys = []
    values = []

    for key in key_value_dict:
        keys.append(key)
        value = key_value_dict[key]
        if not isinstance(value, list):
            value = [value]
        values.append(';'.join(map(str, value)))

    return keys, values
