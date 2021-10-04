// var fs = require('fs')
// const os = require('os')
exports.get_history = (req, res) => {
    return new Promise(async (resolve, reject) => {
        try {
            //분실물 목록을 반환하는 서비스

            //todo 로직 1-1. DB에서 분실물 가져오기

            //로직 1-2. 임시 데이터 
            const history = [{ timestamp: String(new Date()), content: "from nodejs" }, { timestamp: String(new Date()), content: "from nodejs2" }, { timestamp: String(new Date()), content: "from nodejs3" }]

            return resolve(history)
        } catch (error) {
            return reject(error);
        }
    })
}
exports.post_history = (req, res) => {
    return new Promise(async (resolve, reject) => {
        try {
            //분실물 목록을 반환하는 서비스

            //todo 로직 1-1. DB에 분실물 저장하기

            return resolve({})
        } catch (error) {
            return reject(error);
        }
    })
}