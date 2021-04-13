import os
import random

with open(os.path.normpath("utils/pedsToHashes.txt"), "r") as file:
    pedsToHashes_ls = file.read()

pedsToHashes_ls = pedsToHashes_ls.split("\n")
pedsToHashes_ls = [d.split("=") for d in pedsToHashes_ls]

pedsToHashes = {n: h for n, h in pedsToHashes_ls}
hashesToPeds = {h: n for n, h in pedsToHashes_ls}


def convertHashToModelName(hash):
    if isinstance(hash, str):
        pass
    elif isinstance(hash, int):
        hash = "{0:#0{1}x}".format(hash,10)
    else:
        raise ValueError("hash has to be given in type str or int")
    return hashesToPeds[hash]


def convertModelNameToHash(modelName):
    return pedsToHashes[modelName][2:]

def getRandomPed():
    return random.sample(pedsToHashes.keys(), 1)[0]
