import Blender
import bpy

trans = Blender.Mathutils.TranslationMatrix(Blender.Mathutils.Vector([-875000, -458000, 0.0]))
ident = trans.copy()
ident.identity()

if __name__ == '__main__':
    scn = bpy.data.scenes.active

    for o in scn.objects:
        o.setMatrix(ident)
        me = o.getData(False, True)
        me.transform(trans)
        me.update()
