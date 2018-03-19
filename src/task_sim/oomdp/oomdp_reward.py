#!/usr/bin/env python

def reward(state):
    if 'apple_on_box' in state.relations and 'apple_level_with_box' in state.relations \
            and 'lid_closing_box':
        return 0

    elif 'apple_on_box' in state.relations and 'apple_level_with_box' in state.relations:
        return -.01

    else:
        return -.1
