const io = require('socket.io')(3000)

// initialize GUI connection
io.on('connection', socket => {
    console.log('Client successfully connected')
    socket.emit('hello', 'Hello GUI')
    socket.on('forward', data => {
        console.log('move forward')
    })
})


/*

const tls = require('tls')
const fs = require('fs')
// const serverName = '10.42.0.223'
const serverName = 'localhost'
const portNumber = 5000

const options = {
    cert: 'laptop.key',
    ca: 'laptop.crt'
    // rejectUnauthorized: false
}

let socket = tls.connect(portNumber, serverName, options, () => {
    console.log('client connection', socket.authorized ? 'authorized' : 'unauthorized')
    process.stdin.pipe(socket)
    process.stdin.resume()
})

socket.setEncoding('utf8')

socket.on('data', (data) => {
    console.log(data)
})


socket.on('end', () => {
    console.log('ended')
})
*/