import os
import random

with open(os.path.normpath("utils/pedsToHashes.txt"), "r") as file:
    pedsToHashes_ls = file.read()

pedsToHashes_ls = pedsToHashes_ls.split("\n")
pedsToHashes_ls = [d.split("=") for d in pedsToHashes_ls]

pedsToHashes = {n: h for n, h in pedsToHashes_ls}
hashesToPeds = {h: n for n, h in pedsToHashes_ls}

spawnablePeds_ls = [k for k in pedsToHashes.keys() if k[-2:] != "_p"]


def convertHashToModelName(hash):
    if isinstance(hash, str):
        if hash[:2] != "0x":
            hash = "0x" + hash
    elif isinstance(hash, int):
        hash = "{0:#0{1}x}".format(hash,10)
    else:
        raise ValueError("hash has to be given in type str or int")
    
    try:
        modelName = hashesToPeds[hash]
    except KeyError:
        modelName = "UNKNOWN"
    return modelName

def convertModelNameToHash(modelName):
    return pedsToHashes[modelName][2:]

def getRandomPed():
    return random.sample(spawnablePeds_ls, 1)[0]



