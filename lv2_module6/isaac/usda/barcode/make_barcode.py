"""Generate mission barcode textures (Code128, P2-01..04) and barcode.usda.

Paper piece: bars 65 mm wide, white quiet zone around, label under the bars.
Run with a python that has zxing-cpp + Pillow (same as perception venv).
"""
from pathlib import Path

import zxingcpp
from PIL import Image, ImageDraw, ImageFont

CODES = ["P2-01", "P2-02", "P2-03", "P2-04"]
BAR_W, BAR_H = 65.0, 16.0          # bar area [mm]
MARGIN, TOP, GAP, TEXT_H, BOTTOM = 8.0, 6.0, 2.0, 4.0, 4.0  # [mm]
PAPER_W = BAR_W + 2 * MARGIN
PAPER_H = TOP + BAR_H + GAP + TEXT_H + BOTTOM
PX_PER_MODULE = 16
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
HERE = Path(__file__).parent


def modules(text):
    """Code128 bar pattern as a list of bools (True = black), no quiet zone."""
    img = zxingcpp.create_barcode(text, zxingcpp.BarcodeFormat.Code128).to_image(scale=1, add_quiet_zones=False)
    row = Image.fromarray(img.__array__() if hasattr(img, "__array__") else img).convert("L")
    return [row.getpixel((x, 0)) < 128 for x in range(row.width)]


def texture(text):
    mods = modules(text)
    px_mm = len(mods) * PX_PER_MODULE / BAR_W  # integer pixels per module -> crisp bars
    W, H = round(PAPER_W * px_mm), round(PAPER_H * px_mm)
    im = Image.new("L", (W, H), 255)
    d = ImageDraw.Draw(im)
    x0, y0, y1 = round(MARGIN * px_mm), round(TOP * px_mm), round((TOP + BAR_H) * px_mm)
    for i, black in enumerate(mods):
        if black:
            d.rectangle([x0 + i * PX_PER_MODULE, y0, x0 + (i + 1) * PX_PER_MODULE - 1, y1], fill=0)
    font = ImageFont.truetype(FONT, round(TEXT_H * px_mm))
    d.text((x0, y1 + round(GAP * px_mm)), text, fill=0, font=font)
    im.save(HERE / f"{text}.png")
    return len(mods)


USDA = '''#usda 1.0
(
    defaultPrim = "barcode"
    metersPerUnit = 1
    upAxis = "Z"
)

# Mission barcode paper ({w:.0f} x {h:.0f} mm, bars 65 mm wide), Code128.
# Origin = paper center, paper in the YZ plane, printed side faces +X.
# Pick the code with the "code" variant set (default P2-01).
def Xform "barcode" (
    variants = {{
        string code = "P2-01"
    }}
    prepend variantSets = "code"
)
{{
    def Mesh "paper" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {{
        uniform bool doubleSided = 0
        int[] faceVertexCounts = [4]
        int[] faceVertexIndices = [0, 1, 2, 3]
        normal3f[] normals = [(1, 0, 0), (1, 0, 0), (1, 0, 0), (1, 0, 0)] (interpolation = "vertex")
        point3f[] points = [(0, -{hw}, -{hh}), (0, {hw}, -{hh}), (0, {hw}, {hh}), (0, -{hw}, {hh})]
        texCoord2f[] primvars:st = [(0, 0), (1, 0), (1, 1), (0, 1)] (interpolation = "vertex")
        uniform token subdivisionScheme = "none"
        rel material:binding = </barcode/Looks/paper>
    }}

    def Scope "Looks"
    {{
        def Material "paper"
        {{
            token outputs:surface.connect = </barcode/Looks/paper/Surface.outputs:surface>

            def Shader "Surface"
            {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor.connect = </barcode/Looks/paper/Tex.outputs:rgb>
                float inputs:roughness = 0.9
                float inputs:metallic = 0
                token outputs:surface
            }}

            def Shader "Tex"
            {{
                uniform token info:id = "UsdUVTexture"
                token inputs:sourceColorSpace = "sRGB"
                float2 inputs:st.connect = </barcode/Looks/paper/St.outputs:result>
                float3 outputs:rgb
            }}

            def Shader "St"
            {{
                uniform token info:id = "UsdPrimvarReader_float2"
                string inputs:varname = "st"
                float2 outputs:result
            }}
        }}
    }}
    variantSet "code" = {{
{variants}
    }}
}}
'''

VARIANT = '''        "{c}" {{
            over "Looks"
            {{
                over "paper"
                {{
                    over "Tex"
                    {{
                        asset inputs:file = @./{c}.png@
                    }}
                }}
            }}
        }}'''

if __name__ == "__main__":
    for c in CODES:
        print(c, texture(c), "modules")
    (HERE / "barcode.usda").write_text(USDA.format(
        w=PAPER_W, h=PAPER_H, hw=PAPER_W / 2000, hh=PAPER_H / 2000,
        variants="\n".join(VARIANT.format(c=c) for c in CODES)))
