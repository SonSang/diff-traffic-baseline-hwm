import Blender
import bpy


if __name__ == '__main__':
    fi = open("/playpen/sewall/traffic/output.txt")
    scn = bpy.data.scenes.active
    assert(len(scn.objects.selected) == 1)
    obj = scn.objects.selected[0]
    assert(obj.type == 'Mesh')
    me = obj.getData(False, True)
    for ct, line in enumerate(fi):
        pts = [float(x) for x in line.rstrip().split()]
        newobj = scn.objects.new(me)
        newobj.name = "car" + str(ct)
        newobj.setMatrix(Blender.Mathutils.Matrix([pts[3],  pts[4], 0.0, 0.0],
                                                  [pts[4], -pts[3], 0.0, 0.0],
                                                  [0.0,        0.0, 1.0, 0.0],
                                                  [pts[0], pts[1], pts[2],    1.0]))
