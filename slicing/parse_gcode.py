import os
import numpy as np
import pygcode
from pygcode import Machine, Line

# constants
PART = 'leak_test_thin'
GCODE = 'leak_test_mod_thin'
# PART = 'radial_seg_thin'
# GCODE = 'radial_slice_new_origin_thin'
# PART = 'casing_scaled'
# GCODE = 'engine_casing_1'

def main():
    gcode_path = f'{PART}/{GCODE}.NC'

    #### offsets ####
    X_SET = 0
    Y_SET = 0
    Z_SET = 76.2

    # trailing decimals do not parse.
    # fix_trailing_decimals(gcode_path, clean_gcode)

    curr_x = 0.0
    curr_y = 0.0
    curr_z = 0.0
    curr_a = 0.0
    curr_c = 0.0
    



    # log position values
    x = []
    y = []
    z = []
    a = []
    c = []

    # track layer and segment number
    layer = 0
    segment = 0

    # flags for when the layer is changing
    seg_flag = False
    layer_flag = False

    # load gcode
    with open(gcode_path, 'r') as f:
        for line in f:
            if not (
                line.startswith(';') or
                line.startswith('//') or
                line.startswith('G22') or
                line.startswith('G10')
            ):
                for cmd_str in line.split():
                    # parse comments
                    if cmd_str.startswith('//'): 
                        break
                    cmd_letter = cmd_str[0]
                    # print(cmd_letter)
                    cmd_val = cmd_str[1:]
                    # print(cmd_val)
                    # check valid commands
                    if cmd_letter == 'X':
                        curr_x = float(cmd_val)
                    if cmd_letter == 'Y':
                        curr_y = float(cmd_val)
                    if cmd_letter == 'Z':
                        curr_z = float(cmd_val)
                    if cmd_letter == 'A':
                        curr_a = float(cmd_val)
                    elif cmd_letter == 'C':
                        curr_c = float(cmd_val)
    # TODO: need to make sure M commands are detected properly 
                    elif cmd_letter == 'M':
                        # detect between layer transisions or between segment transitions
                        if int(float(cmd_val))==20: # layer change
                            print("layer change")
                            layer_flag = True
                        elif int(float(cmd_val)) == 12: # laser turning off, indicates rapid
                            print("Laser off")
                            seg_flag = True
                        elif int(float(cmd_val)) == 11: # laser turning on, clear lists
                            print("Laser on")
                            x = []
                            y = []
                            z = []
                            a = []
                            c = []
                # update the positions once gcode is processed
                x.append(curr_x + X_SET)
                y.append(curr_y + Y_SET)
                z.append(curr_z + Z_SET)
                a.append(curr_a)
                c.append(curr_c)

                # handle segment end or layer changes
                if seg_flag:
                    data = convert_lists(x,y,z,a,c)
                    np.savetxt(
                        f"{PART}/curve_sliced_relative/slice{layer-1}_{segment}.csv",
                        data,
                        delimiter=','
                    )
                    # reset to recieve next segment
                    x = []
                    y = []
                    z = []
                    a = []
                    c = []
                    segment += 1
                    seg_flag = False
                if layer_flag:
                    data = convert_lists(x,y,z,a,c)
                    # reset to recieve next segment
                    x = []
                    y = []
                    z = []
                    a = []
                    c = []
                    segment = 0
                    layer += 1
                    layer_flag = False


        # data = convert_lists(x,y,z,a,c)
        # os.makedirs(f"{PART}/curve_sliced/", exist_ok=True)
        # np.savetxt(f"{PART}/curve_sliced/raw.csv",data, delimiter=',')

def convert_lists(x,y,z,a,c):
    a = np.deg2rad(np.array(a))
    c = np.deg2rad(c)

    v_y = -np.sin(a)*np.cos(c)
    v_x = -np.sin(a)*np.sin(c)
    v_z = -np.cos(a)

    # convert to one big numpy array and save
    data = np.zeros((len(x),6))
    data[:,0] = x
    data[:,1] = y
    data[:,2] = z
    data[:,3] = v_x
    data[:,4] = v_y
    data[:,5] = v_z

    return data


if __name__=='__main__':
    main()



