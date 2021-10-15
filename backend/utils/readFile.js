const path = require("path") //파일 경로 명시, 인코딩을 제공하는 라이브러리
const { readFileSync, readFile } = require("fs") //file discriptor를 생성하기 위한 라이브러리

//path.resolve = 파일 경로를 os 포맷에 맞게 생성해주는 함수
readFile = function () {
    p = readFileSync(path.resolve(__dirname, "backend", "assets", "map.txt"), "utf-8")

    //map 
    p = p.split("\n")
    for (let i = 0; i < p.length; i++) {
        p[i] = p[i].split(' ')
    }
    return p
}

module.exports = readFile