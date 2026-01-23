"CAD logic to create parts for my modular shelving project"
from dataclasses import dataclass
from pathlib import Path
from typing import cast, Iterable
import copy
import click
from ocp_vscode import show, ColorMap
import build123d as b
from numpy import (rad2deg, atan2)

__all__ = [
    "BoxDimensions",
    "DesignParameters",
    "BOX_DIMS",
    "PARAMS",
    "hex_nut",
    "basic_bracket",
    "rail_sketch",
    "rail_part"
]

# ##### Part ID codes ######
# TBC - Test Bracket with slot colinear with joining axis
# TB - Test Bracket with slot perpendicular to joining axis
# TR - Test Rail
# UB - Universal bracket (rear)


@dataclass
class BoxDimensions:
    """Dimensions of the box to be shelved."""
    width: float = 224
    depth: float = 206
    height: float = 73


@dataclass
class ShelfDimensions:
    """ Dimensions of a single shelf that holds a box."""
    padding: float = 5
    width: float = BoxDimensions.width + 2 * padding
    depth: float = BoxDimensions.depth
    height: float = BoxDimensions.height + 2 * padding


@dataclass
class DesignParameters:
    """Configuration parameters for the design."""
    plywood_thickness: float = 3
    crossbar_width: float = 8
    rail_width: float = 15
    cant_angle: float = 3
    nut_height: float = 3
    nut_radius: float = 7.8
    bolt_hole_diameter: float = 4.5
    shell_thickness: float = 5
    rail_slot_depth: float = 25
    tolerance: float = 0.12
    min_thickness: float = 0.5
    font_size: float = 7
    laser_kerf: float = 0.1
    text_depth: float = 0.3

    def check(self):
        """Checks if parameters are valid."""
        if PARAMS.shell_thickness - PARAMS.nut_height < (2 * PARAMS.min_thickness):
            raise ValueError("Minimum thickness exceeded.")


@dataclass
class BracketPlacement:
    """Return type for bracket placement calculations."""
    stretcher: b.Vector
    upright: b.Vector
    crossbar: b.Vector


BOX_DIMS = BoxDimensions()
SHELF_DIMS = ShelfDimensions()
PARAMS = DesignParameters()

_NUT_TEMPLATE: b.BuildPart | None = None


def nn[T](obj: T | None) -> T:
    """
    Given a nullable value we expect to be non-null, coerce
    it to the non-null type using an assert to satisfy the
    type checker.
    """
    assert obj is not None
    return obj


def hex_nut() -> b.BuildPart:
    """
    Creates a hex nut model.
    """
    global _NUT_TEMPLATE
    if _NUT_TEMPLATE is None:
        with b.BuildPart() as p:
            with b.BuildSketch():
                b.RegularPolygon(PARAMS.nut_radius / 2, 6)
                b.Circle(PARAMS.bolt_hole_diameter / 2, mode=b.Mode.SUBTRACT)

            b.extrude(amount=PARAMS.nut_height)
        _NUT_TEMPLATE = p

    return copy.copy(_NUT_TEMPLATE)


def basic_bracket(slat_width: float, perpendicular_slot: bool = False, with_label: str | None = None) -> b.BuildPart:
    """
    Creates a mounting bracket for mating plywood slats to a 3D printed part.
    """
    with b.BuildSketch(b.Plane.XY, mode=b.Mode.PRIVATE):
        outer = b.Rectangle(PARAMS.rail_slot_depth,
                            slat_width + PARAMS.shell_thickness*2)
        inner = b.Rectangle(PARAMS.rail_slot_depth,
                            slat_width)
        captive_nut = b.RegularPolygon(
            PARAMS.nut_radius / 2 + PARAMS.tolerance, 6
        )

    captive_nut_points = captive_nut.vertices()

    if perpendicular_slot:
        y_nut_min = captive_nut_points.sort_by(b.Axis.Y)[0].Y
        nut_points_x = captive_nut_points.sort_by(b.Axis.X)
        nut_slot_width = nut_points_x[-1].X - nut_points_x[0].X
        nut_slot_plane = b.Plane.XZ.offset(-y_nut_min)
        extrude_dir = (0, 1, 0)
    else:
        x_nut_min = captive_nut_points.sort_by(b.Axis.X)[0].X
        nut_points_y = captive_nut_points.sort_by(b.Axis.Y)
        nut_slot_width = nut_points_y[-1].Y - nut_points_y[0].Y
        nut_slot_plane = b.Plane.YZ.offset(x_nut_min)
        extrude_dir = (1, 0, 0)

    with b.BuildPart() as p:
        b.extrude(inner, PARAMS.shell_thickness)
        b.extrude(cast(b.Sketch, (outer - inner)),
                  amount=PARAMS.shell_thickness + PARAMS.plywood_thickness)
        b.Hole(PARAMS.bolt_hole_diameter / 2)

        with b.BuildSketch(nut_slot_plane):
            with b.Locations((0, PARAMS.shell_thickness / 2)):
                b.Rectangle(nut_slot_width, PARAMS.nut_height)
                b.offset(amount=PARAMS.tolerance, kind=b.Kind.INTERSECTION)

        b.extrude(amount=PARAMS.rail_slot_depth, dir=extrude_dir, mode=b.Mode.SUBTRACT)

        b.RigidJoint(label="bolt_hole", joint_location=b.Location((0, 0, PARAMS.shell_thickness)))

        if with_label is not None:
            text_face = p.faces().sort_by(b.Axis.Y)[0]

            with b.BuildSketch(text_face):
                text = b.Text(with_label, font_size=PARAMS.font_size,
                              position_on_path=0.8, mode=b.Mode.PRIVATE)
                b.add(text.move(b.Location((0, -PARAMS.font_size*0.12, 0))))

            b.extrude(amount=PARAMS.text_depth)

    return p


def get_bracket_length(bracket: b.BuildPart) -> float:
    """
    Calculates the width of the bracket connection across its mating width.
    """
    edges = bracket.faces().sort_by(b.Axis.X)[-1].edges()
    return edges.sort_by(b.SortBy.LENGTH)[-1].length


def vector_to_loc(vector: b.Vector) -> b.Location:
    """
    Creates a Location from a vector, setting rotation based on the vector's angle.
    """
    return b.Location(vector, rad2deg(atan2(vector.Y, vector.X)))


def vectors_to_locs(vectors: Iterable[b.Vector]) -> list[b.Location]:
    """
    Maps a collection of vectors to Locations using vector_to_loc.
    """
    return [vector_to_loc(v) for v in vectors]


def add_bracket_at_position(template: b.BuildPart, position: b.Location, joint_label: str):
    """
    Adds a bracket to the current build context at the given position and defines a joint.
    """
    part = copy.copy(nn(template.part))
    b.add(cast(b.AddType, position * part))

    b.RigidJoint(label=f"bolt_hole_{joint_label}",
                 joint_location=b.Location(position.position + (0, 0, PARAMS.shell_thickness)))


def build_universal_bracket_body(square_bracket_template: b.BuildPart,
                                 crossbar_template: b.BuildPart,
                                 stretcher_bracket_pos: b.Vector,
                                 upright_bracket_pos: b.Vector,
                                 crossbar_bracket_pos: b.Vector):
    "Construct the main body of a universal bracket."

    stretcher_locs = vectors_to_locs((stretcher_bracket_pos, stretcher_bracket_pos * -1))
    upright_locs = vectors_to_locs((upright_bracket_pos, upright_bracket_pos * -1))
    crossbar_locs = vectors_to_locs((crossbar_bracket_pos,
                                     crossbar_bracket_pos.rotate(b.Axis.Y, 180),
                                     crossbar_bracket_pos.rotate(b.Axis.Z, 180),
                                     crossbar_bracket_pos.rotate(b.Axis.X, 180)))

    joint_names = ("E", "N", "W", "S")

    for i, loc in enumerate(stretcher_locs):
        add_bracket_at_position(square_bracket_template, loc, joint_names[i*2])

    for i, loc in enumerate(upright_locs):
        add_bracket_at_position(square_bracket_template, loc, joint_names[i*2+1])

    for i, loc in enumerate(crossbar_locs):
        add_bracket_at_position(crossbar_template, loc, f"crossbar_{i}")


def compute_ub_bracket_positions(shelf_width: float,
                                 shelf_height: float,
                                 crossbar_width: float,
                                 bracket_width: float,
                                 bracket_depth: float) -> BracketPlacement:
    """
    Calculates the placement vectors for the stretcher, upright, and crossbar components.
    """

    V_theta = b.Vector(shelf_width, shelf_height, 0).normalized()
    ux, uy = V_theta.X, V_theta.Y

    # r_x: The diagonal distance required to clear the Stretcher (X-axis)
    r_x = (bracket_width + crossbar_width * ux) / (2 * uy)

    # r_y: The diagonal distance required to clear the Upright (Y-axis)
    r_y = (bracket_width + crossbar_width * uy) / (2 * ux)

    r_final = max(r_x, r_y)

    if r_x >= r_y:
        x_pos = (bracket_width * ux + crossbar_width) / (2 * uy)
    else:
        x_pos = bracket_depth / 2

    if r_y >= r_x:
        y_pos = (bracket_width * uy + crossbar_width) / (2 * ux)
    else:
        y_pos = bracket_depth / 2

    stretcher = b.Vector(x_pos + bracket_depth / 2, 0)
    upright = b.Vector(0, y_pos + bracket_depth / 2)
    crossbar = V_theta * (r_final + bracket_depth / 2)

    return BracketPlacement(stretcher, upright, crossbar)


def universal_bracket(shelf_width: float,
                      shelf_height: float,
                      square_rail_width: float,
                      crossbar_width: float,
                      with_label: str | None = None) -> b.BuildPart:
    """
    Creates a universal bracket for the rear end of the shelves.
    """
    square_bracket_template = basic_bracket(square_rail_width)
    square_bracket_width = get_bracket_length(square_bracket_template)

    crossbar_template = basic_bracket(crossbar_width)
    crossbar_bracket_width = get_bracket_length(crossbar_template)

    placement = compute_ub_bracket_positions(shelf_width,
                                             shelf_height,
                                             crossbar_width,
                                             square_bracket_width,
                                             PARAMS.rail_slot_depth)

    with b.BuildPart() as p:
        build_universal_bracket_body(square_bracket_template,
                                     crossbar_template,
                                     placement.stretcher,
                                     placement.upright,
                                     placement.crossbar)

        text: b.Text | None = None
        with b.BuildSketch():
            b.Rectangle(
                placement.stretcher.X * 2 - PARAMS.rail_slot_depth,
                square_bracket_width
            )
            b.Rectangle(
                square_bracket_width,
                placement.upright.Y * 2 - PARAMS.rail_slot_depth
            )
            b.Rectangle(
                placement.crossbar.length * 2 - PARAMS.rail_slot_depth,
                crossbar_bracket_width,
                rotation=placement.crossbar.get_signed_angle(placement.stretcher)
            )
            b.Rectangle(
                placement.crossbar.length * 2 - PARAMS.rail_slot_depth,
                crossbar_bracket_width,
                rotation=180 - placement.crossbar.get_signed_angle(placement.stretcher)
            )

            if with_label is not None:
                text = b.Text(with_label, font_size=PARAMS.font_size,
                              position_on_path=0.8, mode=b.Mode.PRIVATE)
                text = text.move(b.Location((0, -PARAMS.font_size*0.12, 0)))

        b.extrude(amount=PARAMS.shell_thickness)

        if text is not None:
            b.extrude(text, amount=PARAMS.text_depth + PARAMS.shell_thickness)

    return p


def rail_sketch(rail_length: float, with_label: str | None = None) -> list[b.BuildSketch]:
    """
    Creates the sketches for the rail part (cut and engrave).
    """
    with b.BuildSketch() as cut:
        b.Rectangle(rail_length, PARAMS.rail_width)
        with b.Locations(
            (-rail_length / 2 + PARAMS.rail_slot_depth / 2, 0, 0),
            (rail_length / 2 - PARAMS.rail_slot_depth / 2, 0, 0)
        ):
            b.Circle(PARAMS.bolt_hole_diameter / 2, mode=b.Mode.SUBTRACT)

        b.offset(amount=PARAMS.laser_kerf, kind=b.Kind.INTERSECTION)

    with b.BuildSketch() as engrave:
        if with_label is not None:
            b.Text(with_label, font_size=PARAMS.font_size)

    return [cut, engrave]


def rail_part(rail_length: float, with_label: str | None = None) -> b.BuildPart:
    """
    Creates the 3D model for the rail.
    """
    with b.BuildPart() as p:
        cut, engrave = rail_sketch(rail_length, with_label)
        b.extrude(cut.sketch, amount=PARAMS.plywood_thickness)

        b.RigidJoint(label="left_bolt",
                     joint_location=b.Location(
                         (-rail_length / 2 + PARAMS.rail_slot_depth / 2, 0, 0)
                         ))
        b.RigidJoint(label="right_bolt",
                     joint_location=b.Location(
                         (rail_length / 2 - PARAMS.rail_slot_depth / 2, 0, 0)
                         ))

        if with_label is not None:
            b.extrude(engrave.sketch.move(
                b.Location((0, 0, PARAMS.plywood_thickness))),
                amount=-0.2, mode=b.Mode.SUBTRACT)

    return p


@click.command()
@click.option("--show/--no-show", "show_preview", default=True, help="Show the model in ocp_vscode.")
@click.option(
    "--output",
    "-o",
    "output_dir",
    type=click.Path(file_okay=False, dir_okay=True, path_type=Path),
    help="Output directory for artifacts.",
)
def main(show_preview: bool, output_dir: Path | None):
    """
    Main entry point for generating and previewing parts.
    """
    PARAMS.check()
    tb = basic_bracket(PARAMS.rail_width, perpendicular_slot=True, with_label="TB3")
    tbc = basic_bracket(PARAMS.rail_width, perpendicular_slot=False, with_label="TBC1")
    ub = universal_bracket(SHELF_DIMS.width, SHELF_DIMS.height, PARAMS.rail_width, PARAMS.crossbar_width, "UB1")
    ux = universal_bracket(SHELF_DIMS.height, SHELF_DIMS.width, PARAMS.rail_width, PARAMS.crossbar_width, "UX1")
    u45 = universal_bracket(SHELF_DIMS.width, SHELF_DIMS.width, PARAMS.rail_width, PARAMS.crossbar_width, "U45")

    test_rail_length = SHELF_DIMS.width - 2 * (ub.vertices().sort_by(b.Axis.X)[-1].X - PARAMS.rail_slot_depth)
    rail = rail_part(test_rail_length, "TR2")

    assert (tb.part is not None
            and tbc.part is not None
            and ub.part is not None
            and rail.part is not None
            and ux.part is not None
            and u45.part is not None)

    ub.part.joints["bolt_hole_E"].connect_to(rail.part.joints["left_bolt"])
    rail.part.joints["right_bolt"].connect_to(tb.joints["bolt_hole"])

    assembly = b.Compound([tb.part, rail.part, ub.part])

    arranged_parts = b.pack([assembly, tbc.part, ux.part, u45.part], padding=5, align_z=True)

    if show_preview:
        show(*b.Compound(arranged_parts).solids(), colors=ColorMap.set2())

    if output_dir:
        # Ensure output directory exists
        output_dir.mkdir(parents=True, exist_ok=True)

        sketch_exporter = b.ExportDXF(unit=b.Unit.MM)
        sketch_exporter.add_layer("Engrave", color=b.ColorIndex.BLUE)
        sketch_exporter.add_layer("Cut", color=b.ColorIndex.RED)

        rail_sketches = rail_sketch(test_rail_length, "TR1")
        sketch_exporter.add_shape(rail_sketches[0].sketch, layer="Cut")
        sketch_exporter.add_shape(rail_sketches[1].sketch, layer="Engrave")

        sketch_exporter.write(str(output_dir / "test_rail_1.dxf"))

        b.export_stl(tb.part, str(output_dir / "test_bracket_perpendicular.stl"))  # type: ignore[arg-type]
        b.export_stl(tbc.part, str(output_dir / "test_bracket_collinear.stl"))  # type: ignore[arg-type]


if __name__ == "__main__":
    main()  # pylint: disable=no-value-for-parameter
