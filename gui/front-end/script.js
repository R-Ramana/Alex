const socket = io('http://localhost:3000')
let upButton = document.querySelector('.button-up')
socket.on('hello', data => {
    console.log(data)
})

upButton.addEventListener('click', e => {
    e.preventDefault()
    socket.emit('forward');
})