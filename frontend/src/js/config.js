const NODE_ENV = (process.env.NODE_ENV && (process.env.NODE_ENV).trim().toLowerCase() == 'production') ? 'production' : 'development';
const params = {
    development: {
        // host: "http://localhost:3000"
        host: "http://j5a102.p.ssafy.io:3000"
    },
    production: {
        host: "http://j5a102.p.ssafy.io:3000"
    }
}
export default params[NODE_ENV]