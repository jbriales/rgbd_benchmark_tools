#!/usr/bin/python

import generate_wikitext

datasets = [
            ["Testing and Debugging Datasets",
                ["freiburg1/xyz",
                 "freiburg1/rpy",
                 "freiburg1/floor",
                 "freiburg2/xyz",
                 "freiburg2/rpy"]],
            ["SLAM (Simulatenous Localization and Mapping) Datasets",
                ["freiburg1/desk",
                 "freiburg1/desk2",
                 "freiburg1/room",
                 "freiburg1/360",
                 "freiburg2/desk",
                 "freiburg2/360_hemisphere",
                 "freiburg2/360_kidnap",
                 "freiburg2/large_no_loop",
                 "freiburg2/large_with_loop",
                 "freiburg2/pioneer_360",
                 "freiburg2/pioneer_pioneer2",
                 "freiburg2/pioneer_pioneer3",
                 "freiburg2/pioneer_slam"]],
            ["3D Object Reconstruction Datasets",
                ["freiburg1/plant",
                 "freiburg1/teddy",
                 "freiburg2/coladose",
                 "freiburg2/besteck_teller",
                 "freiburg2/blumen",
                 "freiburg2/blumen_brownbackground",
                 "freiburg2/kugel",
                 "freiburg2/kugel2",
                 ]],
            ["Calibration/Other Datasets",
                ["freiburg1/rgb_calibration",
                 "freiburg1/ir_calibration",
                 "freiburg2/rgb_calibration",
                 "freiburg2/ir_calibration",
                 "freiburg2/large_checkerboard_calibration"
                 ]]            
            ]

files=[]
html = ""
summary = "<table>\n"
for category,sets in datasets:
    print "category '%s'"%category
    summary+="  <tr><td valign='top'><a href='%s'>%s</a></td><td>\n"%(category,category)
    html += "\n<h2><a name='%s'>%s</a></h2>\n"%(category,category)
    for s in sets:
        print "  dataset '%s'"%s
        d = s.split("/")
        bagfile = "/usr/wiss/sturmju/public_html/dirs/rgbd_datasets/" + d[0]+ "/rgbd_dataset_"+d[0]+"_"+d[1]+".bag"
        html += generate_wikitext.generate_wikitext(bagfile,s)
        summary += "    <a href='%s'>%s</a><br>\n"%(s,s)
    summary+="  </td></tr>\n"
summary+="</table>\n"
f = open("/usr/wiss/sturmju/public_html/dirs/rgbd_datasets/index.html","w")
f.write(summary+html)
f.close()