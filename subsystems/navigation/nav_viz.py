
from .env_params import EntityType
from .env_params import entity_info

import glfw
import OpenGL.GL as gl
import imgui

from imgui.integrations.glfw import GlfwRenderer

class NavViz:
  def __init__(self):
    imgui.create_context()
    self.window = impl_glfw_init()
    self.impl = GlfwRenderer(self.window)
    pass

  def update(self):
    should_close = not glfw.window_should_close(self.window)
    glfw.poll_events()
    self.impl.process_inputs()
    return should_close

  def draw(self, env, path, speed, ori, routine):
    scale = 1

    imgui.new_frame()
    imgui.begin("Custom window", True)

    imgui.text('Velocity:         {}'.format(speed))
    imgui.text('Angular Velocity: {}'.format(ori))
    imgui.text('Nav Routine:      {}'.format(routine))

    draw_list  = imgui.get_window_draw_list()
    wnd_pos    = imgui.get_window_position()
    wnd_size   = imgui.get_window_size()
    wnd_size   = imgui.Vec2(wnd_size.x * 0.5, wnd_size.y * 0.5)
    wnd_center = imgui.Vec2(wnd_pos.x + wnd_size.x, wnd_pos.y + wnd_size.y)

    for entity in env.get_group():
      col_raw = entity_info[entity.type()].colour()
      col   = imgui.get_color_u32_rgba(col_raw[0], col_raw[1], col_raw[2], col_raw[3])
      white = imgui.get_color_u32_rgba(1, 1, 1, 1)
      reg     = entity.size(False).x / 2
      reg_inf = entity.size(True).x  / 2
      draw_list.add_circle(entity.position().x * scale + wnd_center[0], scale * entity.position().y + wnd_center[1], reg * scale, col)
      draw_list.add_circle(entity.position().x * scale + wnd_center[0], scale * entity.position().y + wnd_center[1], reg_inf * scale, white)

    if (len(path) > 1):
      for i in range(len(path) - 1):
        draw_list.add_line(path[i][0] * scale + wnd_center[0], path[i][1] * scale + wnd_center[1], path[i + 1][0] * scale + wnd_center[0], path[i + 1][1] * scale + wnd_center[1], imgui.get_color_u32_rgba(1, 1, 1, 1))

    imgui.end()

    #imgui.show_test_window()

    gl.glClearColor(1., 1., 1., 1)
    gl.glClear(gl.GL_COLOR_BUFFER_BIT)

    imgui.render()
    self.impl.render(imgui.get_draw_data())
    glfw.swap_buffers(self.window)
    pass

def impl_glfw_init():
    width, height = 1280, 720
    window_name = "minimal ImGui/GLFW3 example"

    if not glfw.init():
        print("Could not initialize OpenGL context")
        exit(1)

    # OS X supports only forward-compatible core profiles from 3.2
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)

    glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, gl.GL_TRUE)

    # Create a windowed mode window and its OpenGL context
    window = glfw.create_window(
        int(width), int(height), window_name, None, None
    )
    glfw.make_context_current(window)

    if not window:
        glfw.terminate()
        print("Could not initialize Window")
        exit(1)

    return window