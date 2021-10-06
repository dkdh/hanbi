// var fs = require('fs')
// const os = require('os')
var history = [{ timestamp: String(new Date()), content: "from nodejs" }, { timestamp: String(new Date()), content: "from nodejs2" }, { timestamp: String(new Date()), content: "from nodejs3" }]

exports.get_history = (req, res) => {
    return new Promise(async (resolve, reject) => {
        try {
            //로직 1-2. 임시 데이터 

            return resolve(history)
        } catch (error) {
            return reject(error);
        }
    })
}
exports.post_history = (req, res) => {
    return new Promise(async (resolve, reject) => {
        try {
            //todo 로직 1-1. DB에 분실물 저장하기
            const { content } = req.body
            if (!content) throw new Error("형식이 올바르지 않습니다");

            history.push({ timestamp: String(new Date()), content })
            return resolve(history)
        } catch (error) {
            return reject(error.message);
        }
    })
}