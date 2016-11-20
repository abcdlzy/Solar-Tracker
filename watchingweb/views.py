from django.shortcuts import render
from django.http import HttpResponse,HttpResponseRedirect
import datetime
# Create your views here.

def maxLeft(request):
    ff = open('/users/abcdlzy/Downloads/watching/maxLeft', 'w')
    ff.write(request.GET['value'])
    ff.close()
    return HttpResponseRedirect('/')

def maxRight(request):
    ff = open('/users/abcdlzy/Downloads/watching/maxRight', 'w')
    ff.write(request.GET['value'])
    ff.close()
    return HttpResponseRedirect('/')

def index(request):
    maxLeft=float(open('/users/abcdlzy/Downloads/watching/maxLeft').read())
    maxRight=float(open('/users/abcdlzy/Downloads/watching/maxRight').read())
    center=(maxRight+maxLeft)/2
    nowTime=(datetime.datetime.now()+datetime.timedelta(hours=8)).strftime("%Y-%m-%d %H:%M:%S")
    return render(request, 'home.html',{'center':center,'nowTime':nowTime,'maxLeft': maxLeft,'maxRight': maxRight})