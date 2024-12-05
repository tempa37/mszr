<script>
let initialData = {
    mac: '',
    ip: '',
    port: '',
    gateway: ''
};

let adc

function validateData(mac, ip, port, gateway) {
	
	if (mac) {
        // MAC должен быть в формате 6 групп по 2 шестнадцатеричных цифры, разделенные двоеточиями или тире (например, 00:1A:2B:3C:4D:5E)
        const macRegex = /^([0-9A-Fa-f]{2}[:\-]){5}([0-9A-Fa-f]{2})$/;
        if (!macRegex.test(mac)) {
            alert('Неверный формат MAC-адреса.');
            return false;
        }
    }
	
	if (ip) {
        // IP должен быть в формате 4 групп от 0 до 255, разделенных точками (например, 192.168.1.1)
        const ipRegex = /^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
        if (!ipRegex.test(ip)) {
            alert('Неверный формат IP-адреса.');
            return false;
        }
    }
	
	if (port) {
        // Порт должен быть числом от 1 до 65535
        const portNumber = parseInt(port, 10);
        if (isNaN(portNumber) || portNumber < 1 || portNumber > 65535) {
            alert('Неверное значение порта. Порт должен быть в диапазоне от 1 до 65535.');
            return false;
        }
    }
	
	if (gateway) {
	// Шлюз должен быть в формате IP (как и проверка для IP-адреса)
	const gatewayRegex = /^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
	if (!gatewayRegex.test(gateway)) {
		alert('Неверный формат адреса шлюза.');
		return false;
	}
    }
	
	
	
	
	
	
    // Здесь должна быть логика валидации
    return true;
}


function saveChanges() 
{
	 document.getElementById('customModal').style.display = 'block';
}

function closeCustomModal() {
    document.getElementById('customModal').style.display = 'none';
}

function confirmChanges() {
    closeCustomModal();
    saveData(); // Здесь выполняется сохранение данных
}


function saveData() {
    const mac = document.getElementById('field1').value;
    const ip = document.getElementById('field2').value;
    const port = document.getElementById('field3').value;
    const gateway = document.getElementById('field4').value;

    // Объект для хранения измененных данных
    const updatedData = {};

	const changedValues = document.getElementById('changed-values'); //для всплывающего окна

    // Проверяем каждое поле на изменения
    if (mac !== initialData.mac) {
        updatedData.mac = mac;
		changedValues.innerHTML += `<li>MAC: ${mac}</li>`;
    }
    if (ip !== initialData.ip) {
        updatedData.ip = ip;
		changedValues.innerHTML += `<li>IP: ${ip}</li>`;
    }
    if (port !== initialData.port) {
        updatedData.port = port;
		changedValues.innerHTML += `<li>Port: ${port}</li>`;
    }
    if (gateway !== initialData.gateway) {
        updatedData.gateway = gateway;
		changedValues.innerHTML += `<li>Gateway: ${gateway}</li>`;
    }

    // Если нет измененных данных, прерываем выполнение
    if (Object.keys(updatedData).length === 0) {
        alert('Нет измененных данных для сохранения.');
        return;
    }

    // Проверяем валидность измененных данных
    if (!validateData(updatedData.mac, updatedData.ip, updatedData.port, updatedData.gateway)) {
        return; // Если данные невалидны, прерываем выполнение функции
    }

    // Отправляем измененные данные на сервер
    fetch('/save', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify(updatedData)
    })
    .then(response => response.json())
    .then(data => {
        document.getElementById('notification').style.display = 'block'; //уведомление
        // Обновляем начальные значения на сервере
        initialData = { ...updatedData };
    })
    .catch(error => {
        console.error('Ошибка при сохранении данных:', error);
    });
}



function initFields() {
    initialData.mac = document.getElementById('field1').value;
    initialData.ip = document.getElementById('field2').value;
    initialData.port = document.getElementById('field3').value;
    initialData.gateway = document.getElementById('field4').value;
}



// Инициализация начальных значений полей
function loadInitialData() {
    fetch('/get-initial-data')  // Предположим, ваш сервер возвращает дефолтные значения
        .then(response => response.json())
        .then(data => {
            // Установим значения полей
            document.getElementById('field1').value = data.mac || '';
            document.getElementById('field2').value = data.ip || '';
            document.getElementById('field3').value = data.port || '';
            document.getElementById('field4').value = data.gateway || '';

            // Инициализируем объект initialData после получения данных с сервера
            initFields();
        })
        .catch(error => {
            console.error('Ошибка при загрузке данных:', error);
        });
}

// Вызов функции инициализации при загрузке страницы
window.onload = loadInitialData;
</script>