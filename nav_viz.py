
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

  def draw(self, env):
    imgui.new_frame()

    imgui.begin("Custom window", True)
    draw_list  = imgui.get_window_draw_list()
    wnd_pos    = imgui.get_window_position()
    wnd_size   = imgui.get_window_size()
    wnd_size   = imgui.Vec2(wnd_size.x * 0.5, wnd_size.y * 0.5)
    wnd_center = imgui.Vec2(wnd_pos.x + wnd_size.x, wnd_pos.y + wnd_size.y)

    for entity in env.get_entities():
      col = imgui.get_color_u32_rgba(1,1,1,1)

      if   entity.type == 'sample':   col = imgui.get_color_u32_rgba(1, 0.3, 0.3, 1)
      elif entity.type == 'rock':     col = imgui.get_color_u32_rgba(0, 0, 1, 1)
      elif entity.type == 'obstacle': col = imgui.get_color_u32_rgba(0, 1, 0, 1)
      elif entity.type == 'rover':    col = imgui.get_color_u32_rgba(0.3, 0.3, 0.3, 1)
      elif entity.type == 'explore':  col = imgui.get_color_u32_rgba(1, 0, 0, 1)
      draw_list.add_circle(entity.body.position.x * 100 + wnd_center[0], 100 * entity.body.position.y + wnd_center[1], entity.body.radius* 100, col)

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
