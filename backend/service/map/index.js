var fs = require('fs')
const os = require('os')
exports.get_map = (req, res) => {
    return new Promise(async (resolve, reject) => {
        try {
            //! path = project root 폴더 기준 파일 경로
            // const path = "./assets/map.png";

            //파일 읽기
            const data = await fs.readFileSync(path, (error, data) => data)
            return resolve(data)
        } catch (error) {
            return reject(error);
        }
    })
}