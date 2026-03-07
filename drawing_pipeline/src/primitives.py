import numpy as np 

class DrawingProgram: 
    def __init__(self): 
        self.commands = []
    def pen_up(self): 
        self.commands.append(('pen_up',))
    def pen_down(self): 
        self.commands.append(('pen_down',))
    def move_to(self, x, y): 
        self.commands.append(('move_to', x, y))
    def line_to(self, x, y): 
        self.commands.append(('line_to', x, y))
    def circle(self, cx, cy, radius, n_points=64):
        self.commands.append(('circle', cx, cy, radius, n_points))


def sample_program(program, ds_mm=2.0):
    """Convert a DrawingProgram into evenly-spaced waypoints."""
    x_list, y_list, pen_list = [], [], []
    cur_x, cur_y = 0.0, 0.0
    pen_state = 0

    for cmd in program.commands:
        cmd_type = cmd[0]

        if cmd_type == 'pen_up':
            pen_state = 0
        elif cmd_type == 'pen_down':
            pen_state = 1
        elif cmd_type == 'move_to':
            _, tx, ty = cmd
            _interpolate(cur_x, cur_y, tx, ty, 0, ds_mm, x_list, y_list, pen_list)
            cur_x, cur_y = tx, ty
        elif cmd_type == 'line_to':
            _, tx, ty = cmd
            _interpolate(cur_x, cur_y, tx, ty, pen_state, ds_mm, x_list, y_list, pen_list)
            cur_x, cur_y = tx, ty
        elif cmd_type == 'circle':
            _, cx, cy, r, n = cmd
            start_x, start_y = cx + r, cy
            _interpolate(cur_x, cur_y, start_x, start_y, 0, ds_mm, x_list, y_list, pen_list)
            for i in range(1, n + 1):
                angle = 2 * np.pi * i / n
                x_list.append(cx + r * np.cos(angle))
                y_list.append(cy + r * np.sin(angle))
                pen_list.append(pen_state)
            cur_x, cur_y = start_x, start_y

    return np.array(x_list), np.array(y_list), np.array(pen_list)


def _interpolate(x0, y0, x1, y1, pen_val, ds, x_list, y_list, pen_list):
    """Interpolate from (x0,y0) to (x1,y1) with spacing ds."""
    dist = np.hypot(x1 - x0, y1 - y0)
    if dist < 1e-6:
        return
    n_steps = max(1, int(np.ceil(dist / ds)))
    for i in range(1, n_steps + 1):
        t = i / n_steps
        x_list.append(x0 + t * (x1 - x0))
        y_list.append(y0 + t * (y1 - y0))
        pen_list.append(pen_val)

def apply_params(x_mm, y_mm, scale=1.0, rot_deg=0.0, dx_mm=0.0, dy_mm=0.0): 
    """Apply scaling, rotation, and translation to the input coordinates."""
    # center the drawing 
    cx, cy = x_mm.mean(), y_mm.mean()
    x = (x_mm - cx) * scale
    y = (y_mm - cy) * scale
    # Rotate
    theta = np.radians(rot_deg)
    cos_rot = np.cos(theta)
    sin_rot = np.sin(theta)
    x_rot = cos_rot * x - sin_rot * y
    y_rot = sin_rot * x + cos_rot * y
    return x_rot + cx + dx_mm, y_rot + cy + dy_mm