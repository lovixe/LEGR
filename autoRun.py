import os

def changeLink(min, max):
    f = open('./scratch/gpsr-gfdr.cc')
    text = f.read()
    f.close()
    start = text.find('init(3000, ')
    end = text.find(')', start)
    v0 = text[start:end]
    v1 = 'init(3000, ' + str(min) + ',' + str(max)
    print(v0)
    print(v1)
    text = text.replace(v0, v1)
    w = open('./scratch/gpsr-gfdr.cc', 'w')
    w.write(text)
    w.close()

def setSeed(seed):
    f = open('./scratch/gpsr-gfdr.cc')
    text = f.read()
    f.close()
    start = text.find('seed (')
    end = text.find(')', start)
    v0 = text[start:end]
    v1 = 'seed (' + str(seed)
    text = text.replace(v0, v1)
    w = open('./scratch/gpsr-gfdr.cc', 'w')
    w.write(text)
    w.close()

def exeAndSave():
    os.system('./waf --run scratch/gpsr-gfdr')
    #打开文件并统计
    f = open('runResult.txt')
    text = f.read()
    f.close()
    f = open('stat.txt', 'a+')
    f.write(text)
    f.close()

def changePosition(pos):
    f = open('./scratch/gpsr-gfdr.cc')
    text = f.read()
    f.close()
    start = text.find('clientID(')
    end = text.find(')', start)
    v0 = text[start:end]
    text = text.replace(v0, 'clientID(' + str(pos))

    w = open('./scratch/gpsr-gfdr.cc', 'w')
    w.write(text)
    w.close()

def canexit():
    f = open('./scratch/gpsr-gfdr.cc')
    text = f.read()
    f.close()
    if text.find('exit') != -1:
        return True
    else:
        return False

def Test(position, rsgRange, seed):
    f = open('stat.txt','a+')
    f.write(str(position) + ":" + str(rsgRange) + ":" + str(seed) + "\n")
    f.close()
    changePosition(position)
    changeLink(-rsgRange, rsgRange)
    setSeed(seed)
    exeAndSave()



#编写脚本
def doTest():
    position = range(219,220)
    seeds = range(5490,5500)
    rsgRange = [1.5, 2]

    for i in position:
        for j in rsgRange:
            for k in seeds:
                Test(i, j ,k)
                if canexit() == True:
                    return


doTest()