#!/usr/bin/python
import numpy
import os
import math

def list_to_plot(d,options='',legend=''):
  if len(legend)==0:
    forgetplot = 'forget plot,'
    legendcmd = ''
  else:
    forgetplot = ''
    legendcmd = '''
      \\addlegendentry{%s};
'''%legend

  return (
'''
      \\addplot[%s%s] coordinates { %s }; %s
'''%(forgetplot,options," ".join(["(%f,%f)"%(a,b) for (a,b) in d]),legendcmd) )

def list_to_filledplot(d,options='',legend=''):
  if len(legend)==0:
    forgetplot = 'forget plot,'
    legendcmd = ''
  else:
    forgetplot = ''
    legendcmd = '''
      \\addlegendentry{%s};
'''%legend

  return (
'''
      \\addplot[%s%s] coordinates { %s %s }; %s
'''%(forgetplot,options,
     " ".join(["(%f,%f)"%(a,b) for (a,b,c) in d]),
     " ".join(["(%f,%f)"%(a,c) for (a,b,c) in reversed(d)]),legendcmd) )

def list_to_meanstdareaplot(d,options_line='',options_area='',legend_line='',legend_area=''):
  line=[]
  area=[]
  for (x,v) in d:
    m = numpy.mean(v)
    s = numpy.std(v)
    if not math.isnan(m) and not math.isnan(s):
      line.append( (x,m) )
      area.append( (x,m-s,m+s) )

  return ( list_to_filledplot(area,options_area,legend_area) +
           list_to_plot(line,options_line,legend_line) )

def list_to_meanstdareaplot2(d,options_line='',options_area='',legend_line='',legend_area=''):
  line=[]
  area=[]
  for (x,m,s) in d:
    if not math.isnan(m) and not math.isnan(s):
      line.append( (x,m) )
      area.append( (x,m-s,m+s) )

  return ( list_to_filledplot(area,options_area,legend_area) +
           list_to_plot(line,options_line,legend_line) )

def list_to_stdlineplot(d,options_line='',legend_line=''):
  line1=[]
  line2=[]
  for (x,v) in d:
    m = numpy.mean(v)
    s = numpy.std(v)
    if not math.isnan(m) and not math.isnan(s):
      line1.append( (x,m-s) )
      line2.append( (x,m+s) )

  return ( list_to_plot(line1,options_line,legend_line)
           +list_to_plot(line2,options_line,legend_line))

def list_to_meanplot(d,options_line='',legend_line=''):
  line=[]
  area=[]
  for (x,v) in d:
    m = numpy.mean(v)
    s = numpy.std(v)
    if not math.isnan(m) and not math.isnan(s):
      line.append( (x,m) )
      area.append( (x,m-s,m+s) )


  return ( list_to_plot(line,options_line,legend_line) )

def list_to_stdareaplot(d,options_area='',legend_area=''):
  line=[]
  area=[]
  for (x,v) in d:
    m = numpy.mean(v)
    s = numpy.std(v)
    if not math.isnan(m) and not math.isnan(s):
      line.append( (x,m) )
      area.append( (x,m-s,m+s) )

  return ( list_to_filledplot(area,options_area,legend_area) )

def plots_to_axis(plot=[],options=''):
  return (
'''
    \\begin{axis}[%s]
      %s
    \\end{axis}
'''%(options,";\n      ".join(plot)))

def axes_to_file(axes,filename='testfile',preamble_file='\documentclass[11pt]{article}\usepackage{pgf}\n\usepackage{pgfplots}\n\usepackage{tikz}\n\usetikzlibrary{arrows,positioning,fit,shapes,calc,backgrounds,through}\n',pdflatex=True,pdfcrop=False,pdfview=True,pdfconvert=False):
  print 'writing %s.tikz'%filename
  f = open('%s.tikz'%filename,'w')
  f.write('''
%s
\\include{tikzcommands}
\\pagestyle{empty}

\\begin{document}
  \\begin{tikzpicture}
    %s
  \\end{tikzpicture}
\\end{document}
  '''%(preamble_file,"\n    ".join(axes)))
  f.close()
  dir,fname = os.path.split(filename)
  if pdflatex:
    print 'creating %s.pdf'%filename
    os.system('cd %s && yes X 2>/dev/null | pdflatex -interaction=nonstopmode %s.tikz 2>/dev/null >/dev/null'%(dir,fname))
    if pdfcrop:
      os.system('cd %s; pdfcrop %s.pdf %s.pdf >/dev/null'%(dir,fname,fname))
    if pdfview:
      os.system('cd %s; evince %s.pdf'%(dir,fname))
    if pdfconvert:
      os.system('cd %s; convert -density 600x600 -resize %d %s.pdf %s-jpg.jpg'%
          (dir,640,fname,fname ))
