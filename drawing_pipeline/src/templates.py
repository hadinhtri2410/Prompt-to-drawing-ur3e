from primitives import DrawingProgram


def plane():
    """Side-view airplane as one continuous outline, centered near origin. Units: mm."""
    p = DrawingProgram()

    # single continuous stroke tracing the full outline
    p.pen_up()
    p.move_to(80, 0)
    p.pen_down()

    # nose tip curving up into top of fuselage
    p.line_to(90, 5)
    p.line_to(80, 12)

    # top fuselage back to wing root
    p.line_to(20, 14)

    # top wing (swept back)
    p.line_to(-10, 55)
    p.line_to(-25, 50)

    # wing back to fuselage
    p.line_to(0, 14)

    # top fuselage continuing to tail
    p.line_to(-60, 16)

    # tail fin up and back down
    p.line_to(-80, 50)
    p.line_to(-85, 45)
    p.line_to(-70, 18)

    # rear of fuselage
    p.line_to(-75, 16)
    p.line_to(-75, -4)

    # bottom fuselage forward to bottom wing root
    p.line_to(-60, -6)
    p.line_to(0, -4)

    # bottom wing (swept back)
    p.line_to(-25, -40)
    p.line_to(-10, -45)

    # wing back to fuselage
    p.line_to(20, -4)

    # bottom fuselage back to nose
    p.line_to(80, -2)
    p.line_to(80, 0)

    p.pen_up()
    return p


def bike():
    """Side-view bicycle, neon style. Units: mm."""
    p = DrawingProgram()

    # rear wheel
    p.pen_up()
    p.move_to(-45, 0)
    p.pen_down()
    p.circle(-45, 0, 30)

    # front wheel
    p.pen_up()
    p.move_to(45, 0)
    p.pen_down()
    p.circle(45, 0, 30)

    # frame triangle: rear hub → bottom bracket → seat top → rear hub
    p.pen_up()
    p.move_to(-45, 0)
    p.pen_down()
    p.line_to(0, 0)
    p.line_to(-15, 40)
    p.line_to(-45, 0)

    # down tube + fork: bottom bracket → head tube → front hub
    p.pen_up()
    p.move_to(0, 0)
    p.pen_down()
    p.line_to(35, 40)
    p.line_to(45, 0)

    # top tube: seat post top → head tube
    p.pen_up()
    p.move_to(-15, 40)
    p.pen_down()
    p.line_to(35, 40)

    # handlebars
    p.line_to(42, 50)

    # seat
    p.pen_up()
    p.move_to(-22, 42)
    p.pen_down()
    p.line_to(-8, 42)

    p.pen_up()
    return p


def face_simple():
    """Smiley face with glasses, neon style. Units: mm."""
    p = DrawingProgram()

    # head outline
    p.pen_up()
    p.move_to(50, 0)
    p.pen_down()
    p.circle(0, 0, 50)

    # left glasses lens (rectangle)
    p.pen_up()
    p.move_to(-32, 8)
    p.pen_down()
    p.line_to(-32, 22)
    p.line_to(-8, 22)
    p.line_to(-8, 8)
    p.line_to(-32, 8)

    # bridge
    p.pen_up()
    p.move_to(-8, 15)
    p.pen_down()
    p.line_to(8, 15)

    # right glasses lens (rectangle)
    p.pen_up()
    p.move_to(8, 8)
    p.pen_down()
    p.line_to(8, 22)
    p.line_to(32, 22)
    p.line_to(32, 8)
    p.line_to(8, 8)

    # smile (arc)
    p.pen_up()
    p.move_to(-25, -12)
    p.pen_down()
    p.line_to(-18, -22)
    p.line_to(-8, -27)
    p.line_to(0, -28)
    p.line_to(8, -27)
    p.line_to(18, -22)
    p.line_to(25, -12)

    p.pen_up()
    return p


TEMPLATES = {
    "plane": plane,
    "bike": bike,
    "face_simple": face_simple,
}
