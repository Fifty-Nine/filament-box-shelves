"CAD logic to create parts for my modular shelving project"
from dataclasses import dataclass
import copy
from pathlib import Path
import click
from ocp_vscode import show
import build123d as b

__all__ = [
    "BoxDimensions",
    "DesignParameters",
    "BOX_DIMS",
    "PARAMS",
    "hex_nut",
    "basic_bracket",
    "rail_sketch",
    "rail_part",
    "shelf_joiner_a1",
]


@dataclass
class BoxDimensions:
    """Dimensions of the box to be shelved."""
    width: float = 224
    length: float = 206
    height: float = 73


@dataclass
class DesignParameters:
    """Configuration parameters for the design."""
    plywood_thickness: float = 3
    crossbar_width: float = 20
    rail_width: float = 20
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

    def check(self):
        """Checks if parameters are valid."""
        if PARAMS.shell_thickness - PARAMS.nut_height < (2 * PARAMS.min_thickness):
            raise ValueError("Minimum thickness exceeded.")


BOX_DIMS = BoxDimensions()
PARAMS = DesignParameters()

_NUT_TEMPLATE: b.BuildPart | None = None


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


def basic_bracket(slat_width: float, with_label: str | None = None) -> b.BuildPart:
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
        ).rotate(b.Axis.Z, 90)

    captive_nut_points = captive_nut.vertices()
    y_nut_min = captive_nut_points.sort_by(b.Axis.Y)[0].Y
    nut_points_x = captive_nut_points.sort_by(b.Axis.X)
    nut_slot_width = nut_points_x[-1].X - nut_points_x[0].X
    nut_slot_plane = b.Plane.XZ.offset(-y_nut_min)

    with b.BuildPart() as p:
        b.extrude(inner, PARAMS.shell_thickness)
        b.extrude((outer - inner),
                  PARAMS.shell_thickness + PARAMS.plywood_thickness)
        b.Hole(PARAMS.bolt_hole_diameter / 2)

        with b.BuildSketch(nut_slot_plane):
            with b.Locations((0, PARAMS.shell_thickness / 2)):
                b.Rectangle(nut_slot_width, PARAMS.nut_height)
                b.offset(amount=PARAMS.tolerance, kind=b.Kind.INTERSECTION)

        b.extrude(amount=PARAMS.rail_slot_depth, dir=(0, 1, 0), mode=b.Mode.SUBTRACT)

        if with_label is not None:
            text_face = p.faces().sort_by(b.Axis.Y)[0]

            with b.BuildSketch(text_face):
                text = b.Text(with_label, font_size=PARAMS.font_size,
                              position_on_path=0.8, mode=b.Mode.PRIVATE)
                b.add(text.move(b.Location((0, -PARAMS.font_size*0.12, 0))))

            b.extrude(amount=0.3)

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

        if with_label is not None:
            b.extrude(engrave.sketch.move(
                b.Location((0, 0, PARAMS.plywood_thickness))),
                amount=-0.2, mode=b.Mode.SUBTRACT)

    return p


def shelf_joiner_a1() -> b.BuildPart:
    """
    Creates the model for the rear end caps that hold the shelves together.
    """
    with b.BuildPart() as p:
        b.add(basic_bracket(PARAMS.rail_width))

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
    test_bracket = basic_bracket(PARAMS.rail_width, "TB2")

    test_rail_length = 100

    rail = rail_part(test_rail_length, "TR1")
    rail_moved = rail.part.move(  # type: ignore[union-attr]
        b.Location((PARAMS.rail_slot_depth / 2, 0, PARAMS.shell_thickness))
    )

    if show_preview:
        show([test_bracket, rail_moved])

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

        b.export_stl(test_bracket.part, str(output_dir / "test_bracket.stl"))  # type: ignore[arg-type]


if __name__ == "__main__":
    main()  # pylint: disable=no-value-for-parameter
