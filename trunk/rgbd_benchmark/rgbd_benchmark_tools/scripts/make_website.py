#!/usr/bin/python

import generate_wikitext

datasets = [
            ["Testing and Debugging Datasets",
                ["freiburg1/xyz",
                 "freiburg1/rpy",
                 "freiburg1/floor",
                 "freiburg2/xyz",
                 "freiburg2/rpy"]],
            ["SLAM Evaluation Datasets",
                [
                 "freiburg1/360",
                 "freiburg2/360_hemisphere",
                 "freiburg2/360_kidnap",
                 "freiburg1/desk",
                 "freiburg1/desk2",
                 "freiburg2/desk",
                 "freiburg2/desk_with_person",
                 "freiburg1/room",
                 "freiburg2/large_no_loop",
                 "freiburg2/large_with_loop",
                 "freiburg2/pioneer_360",
                 "freiburg2/pioneer_slam",
                 "freiburg2/pioneer_slam2",
                 "freiburg2/pioneer_slam3"
                 ]],
            ["3D Object Reconstruction Datasets",
                ["freiburg1/plant",
                 "freiburg1/teddy",
                 "freiburg2/coke",
                 "freiburg2/dishes",
                 "freiburg2/flowerbouquet",
                 "freiburg2/flowerbouquet_brownbackground",
                 "freiburg2/metallic_sphere",
                 "freiburg2/metallic_sphere2",
                 ]],
            ["Calibration Datasets",
                ["freiburg1/rgb_calibration",
                 "freiburg1/ir_calibration",
                 "freiburg2/rgb_calibration",
                 "freiburg2/ir_calibration",
                 "freiburg2/large_checkerboard_calibration"
                 ]]            
            ]

files=[]
html = ""
summary = '''<div class="toc"> 
<div class="tocheader toctoggle" id="toc__header">Table of Contents</div> 
<div id="toc__inside"> 
<ul class="toc">\n'''
for category,sets in datasets:
    print "category '%s'"%category
    summary+='<li class="level1"><div class="li"><span class="li"><a href="#%s" class="toc">%s</a></span></div>'%(category,category)
    summary+='<ul class="toc"> '
    html += "\n<h2><a name='%s'>%s</a></h2>\n"%(category,category)
    for s in sets:
        print "  dataset '%s'"%s
        d = s.split("/")
        bagfile = "/usr/wiss/sturmju/public_html/dirs/rgbd_datasets/" + d[0]+ "/rgbd_dataset_"+d[0]+"_"+d[1]+".bag"
        html += generate_wikitext.generate_wikitext(bagfile,s)
        summary+='<li class="level2"><div class="li"><span class="li"><a href="#%s" class="toc">%s</a></span></div></li>'%(d[0]+"_"+d[1],d[0]+"_"+d[1])
    summary+="</li></ul>"
summary+="</ul></div></div>"
f = open("/usr/wiss/sturmju/public_html/dirs/rgbd_datasets/index.html","w")
f.write(html)
f.close()
f = open("/usr/wiss/sturmju/public_html/dirs/rgbd_datasets/toc.html","w")
f.write(summary)
f.close()
