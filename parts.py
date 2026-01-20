"CAD logic to create parts for my modular shelving project"
from ocp_vscode import show
import build123d as b
from dataclasses import dataclass
import copy

@dataclass
class BoxDimensions:
    width: float = 224
    length: float = 206
    height: float = 73

@dataclass
class DesignParameters:
    plywood_thickness: float = 3
    crossbar_width: float = 20
    rail_width: float = 20
    cant_angle: float = 3
    nut_height: float = 3
    nut_radius: float = 7.8
    bolt_hole_diameter: float = 4.5
    shell_thickness: float = 5
    rail_slot_depth: float = 25
    tolerance: float = 0.2
    min_thickness: float = 0.5
    font_size: float = 7
    laser_kerf: float = 0.1

bdims = BoxDimensions()
params = DesignParameters()


def check_params():
    if params.shell_thickness - params.nut_height < (2 * params.min_thickness):
        raise Exception("Shell thickness too small to allow fully captive nut.")

check_params()

nut_template: b.BuildPart | None = None

def hex_nut() -> b.BuildPart:
    global nut_template

    if nut_template is not None:
        return copy.copy(nut_template)

    with b.BuildPart() as p:
        with b.BuildSketch():
            b.RegularPolygon(params.nut_radius / 2, 6)
            b.Circle(params.bolt_hole_diameter / 2, mode=b.Mode.SUBTRACT)

        b.extrude(amount=params.nut_height)

    nut_template = p
    return copy.copy(p)

def basic_bracket(slat_width: float, with_label: str | None = None) -> b.BuildPart:
    """
    Creates a mounting bracket for mating plywood slats to a 3D printed part.
    """
    with b.BuildSketch(b.Plane.XY, mode=b.Mode.PRIVATE):
        outer = b.Rectangle(params.rail_slot_depth,
                            slat_width + params.shell_thickness*2)
        inner = b.Rectangle(params.rail_slot_depth,
                            slat_width)
        captive_nut = b.RegularPolygon(params.nut_radius / 2 + params.tolerance, 6).rotate(b.Axis.Z, 90)

    captive_nut_points = captive_nut.vertices()
    y_nut_min = captive_nut_points.sort_by(b.Axis.Y)[0].Y
    nut_slot_width = captive_nut_points.sort_by(b.Axis.X)[-1].X - captive_nut_points.sort_by(b.Axis.X)[0].X
    nut_slot_plane = b.Plane.XZ.offset(-y_nut_min)

    with b.BuildPart() as p:
        b.extrude(inner, params.shell_thickness)
        b.extrude((outer - inner),
                  params.shell_thickness + params.plywood_thickness)
        b.Hole(params.bolt_hole_diameter / 2)

        with b.BuildSketch(nut_slot_plane):
            with b.Locations((0, params.shell_thickness / 2)):
                b.Rectangle(nut_slot_width, params.nut_height)
                b.offset(amount=params.tolerance, kind=b.Kind.INTERSECTION)


        b.extrude(amount=params.rail_slot_depth, dir=(0, 1, 0), mode=b.Mode.SUBTRACT)

        if with_label is not None:
            text_face = p.faces().sort_by(b.Axis.Y)[0]

            with b.BuildSketch(text_face):
                text = b.Text(with_label, font_size=params.font_size, position_on_path=0.8, mode=b.Mode.PRIVATE)
                b.add(text.move(b.Location((0, -params.font_size*0.12, 0))))

            b.extrude(amount=0.3)


    return p

def rail_sketch(rail_length: float, with_label: str | None = None) -> list[b.BuildSketch]:
    with b.BuildSketch() as cut:
        b.Rectangle(rail_length, params.rail_width)
        with b.Locations(
            (-rail_length / 2 + params.rail_slot_depth / 2, 0, 0),
            (rail_length / 2 - params.rail_slot_depth / 2, 0, 0)
        ):
            b.Circle(params.bolt_hole_diameter / 2, mode=b.Mode.SUBTRACT)

    with b.BuildSketch() as engrave:
        if with_label is not None:
            b.Text("TR1", font_size=params.font_size)

    return [cut, engrave]

def rail_part(rail_length: float, with_label: str | None = None) -> b.BuildPart:
    with b.BuildPart() as p:
        cut, engrave = rail_sketch(rail_length, with_label)
        b.extrude(cut.sketch, amount=params.plywood_thickness)

        if with_label is not None:
            b.extrude(engrave.sketch.move(b.Location((0, 0, params.plywood_thickness))), amount=-0.2, mode=b.Mode.SUBTRACT)

    return p


def shelf_joiner_a1() -> b.BuildPart:
    """
    Creates the model for the rear end caps that hold the shelves together.
    """
    with b.BuildPart() as p:
        b.add(basic_bracket(params.rail_width))

    return p


test_bracket = basic_bracket(params.rail_width, "TB2")

test_rail_length = 100

rail = rail_part(test_rail_length, "TR1")
rail.part.move(b.Location((-0*test_rail_length + params.rail_slot_depth / 2, 0, params.shell_thickness)))

show([test_bracket, rail])

sketch_exporter = b.ExportDXF(unit=b.Unit.MM)
sketch_exporter.add_layer("Engrave", color=b.ColorIndex.BLUE)
sketch_exporter.add_layer("Cut", color=b.ColorIndex.RED)

rail_sketch = rail_sketch(test_rail_length, "TR1")
sketch_exporter.add_shape(rail_sketch[0].sketch, layer="Cut")
sketch_exporter.add_shape(rail_sketch[1].sketch, layer="Engrave")

sketch_exporter.write("test_rail_1.dxf")

b.export_stl(test_bracket.part, 'test_bracket.stl')