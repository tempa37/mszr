

let initialData = {
    mac: '',
    ip: '',
    mask: '',
    gateway: '',
	pin: ''
};

function extractValue(value) {
	return value.replace(/<!--.*?-->/, '').trim(); // Убираем шаблон и лишние пробелы
}

window.onload = function() {
	
    var inputField1 = document.getElementById("field3");
    if (inputField1) {
        inputField1.value = inputField1.value.replace("<!--#MASK-->", "");
    }
	
	var inputField4 = document.getElementById("field1");
    if (inputField4) {
        inputField4.value = inputField4.value.replace("<!--#MAC-->", "");
    }
	
	var inputField2 = document.getElementById("field2");
    if (inputField2) {
        inputField2.value = inputField2.value.replace("<!--#IP-->", "");
    }
	
	var inputField3 = document.getElementById("field4");
    if (inputField3) {
        inputField3.value = inputField3.value.replace("<!--#GETAWEY-->", "");
    }
	var inputField6 = document.getElementById("field6");
    if (inputField6) {
        inputField6.value = inputField6.value.replace("<!--#SERIAL-->", "");
    }
	
	var inputField8 = document.getElementById("field8");
    if (inputField8) {
        inputField8.value = inputField8.value.replace(/<!--#\w+-->/g, "");
    }
	
		var inputField21 = document.getElementById("field20");
    if (inputField21) {
        inputField21.value = inputField21.value.replace("<!--#SPEED-->", "");
    }
		var inputField22 = document.getElementById("field21");
    if (inputField22) {
        inputField22.value = inputField22.value.replace("<!--#PARITY-->", "");
    }
		var inputField23 = document.getElementById("field22");
    if (inputField23) {
        inputField23.value = inputField23.value.replace("<!--#STOPB-->", "");
    }
	
	var inputField9 = document.getElementById("field9");
    if (inputField9) {
        inputField9.value = inputField9.value.replace("<!--#SERIAL-->", "");
    }
	
	var inputField10 = document.getElementById("field10");
    if (inputField10) {
        inputField10.value = inputField10.value.replace("<!--#SOFT-->", "");
    }
	
	var inputField11 = document.getElementById("field11");
    if (inputField11) {
        inputField11.value = inputField11.value.replace(/<!--#\w+-->/g, "");
    }
	
	initialData.mask = document.getElementById('field3').value;
	initialData.ip = document.getElementById('field2').value;
	initialData.gateway = document.getElementById('field4').value;

	changeButtonTextBasedOnResolution();


	var inputField99 = document.getElementById("field20");
	if(inputField99)
	{

		// Устанавливаем значение скорости RS-485
		const defaultSpeed = extractValue(document.getElementById('field20').value);
		const speedSelect = document.getElementById('field4-speed');
		speedSelect.value = defaultSpeed; // Устанавливаем значение в select

		// Устанавливаем значение стоп-бита
		const defaultStopBit = document.getElementById('field22').value;
		const stopbitSelect = document.getElementById('field4-stopbit');
		stopbitSelect.value = defaultStopBit; // Устанавливаем значение в select

		// Устанавливаем значение четности
		const defaultParity = document.getElementById('field21').value;
		const paritySelect = document.getElementById('field4-parity');
		paritySelect.value = defaultParity; // Устанавливаем значение в select
		
		
		initialData.rs485speed = defaultSpeed;
		initialData.rs485stopbit = defaultStopBit;
		initialData.rs485parity = defaultParity;;
		
		
		console.log("Speed value:", speedSelect.value);
		console.log("Stop Bit value:", document.getElementById('field22').value);
		console.log("Parity value:", document.getElementById('field21').value);
	}

	var errorPin = document.getElementById('field5').value;
	if(errorPin)
	{
	errorPin = errorPin.replace("<!--#PIN-->", "").trim();
	if (errorPin.trim() == "0") 
	{
		document.getElementById("main-content").style.display = "none"; // Скрыть основную страницу
		document.getElementById("error-page").style.display = "block";  // Показать страницу с ошибкой

		setTimeout(function() 
		{
			window.location.href = "/index.shtml"; // Перенаправление 
		}, 5000);
	}
	setTimeout(function() 
		{
			window.location.href = `http://${initialData.ip}`; // Перенаправление 
		}, 35000);
	}
	
};

function validateData( ip, mask, gateway) {
	

	if (ip) {
        const ipRegex = /^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
        if (!ipRegex.test(ip)) {
            alert('Неверный формат IP-адреса.');
            return false;
        }
    }
	
	if (mask) {
        const maskRegex = /^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
        if (!maskRegex.test(mask)) {
            alert('Неверный формат маски.');
            return false;
        }
    }
	
	if (gateway) {
		const gatewayRegex = /^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
	if (!gatewayRegex.test(gateway)) {
		alert('Неверный формат адреса шлюза.');
		return false;
	}
    }
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
    saveData(); 
}
function invaliddata() 
{
	 document.getElementById('invalid').style.display = 'block';
}
function closemodal()
{
	    document.getElementById('invalid').style.display = 'none';
}

function saveData() {
	

    const ip = document.getElementById('field2').value;
    const mask = document.getElementById('field3').value;
    const gateway = document.getElementById('field4').value;
	const pin = document.getElementById('field5').value;
	const rs485speed = document.getElementById('field4-speed').value;
	const rs485stopbit = document.getElementById('field4-stopbit').value;
	const rs485parity = document.getElementById('field4-parity').value;
	

    const updatedData = {};
	const changedValues = document.getElementById('changed-values'); //для всплывающего окна


    if (ip !== initialData.ip) {
        updatedData.ip = ip;
		changedValues.innerHTML += `<li>IP: ${ip}</li>`;
    }
    if (mask !== initialData.mask) {
        updatedData.mask = mask;
		changedValues.innerHTML += `<li>Mask: ${mask}</li>`;
    }
	
	
	if (rs485speed !== initialData.rs485speed) {
        updatedData.rs485speed = rs485speed;
		changedValues.innerHTML += `<li>rs485speed: ${rs485speed}</li>`;
    }
	
	if (rs485stopbit !== initialData.rs485stopbit) {
        updatedData.rs485stopbit = rs485stopbit;
		changedValues.innerHTML += `<li>rs485stopbit: ${rs485stopbit}</li>`;
    }
	
	if (rs485parity !== initialData.rs485parity) {
        updatedData.rs485parity = rs485parity;
		changedValues.innerHTML += `<li>rs485parity: ${rs485parity}</li>`;
    }
	
	
	
    if (gateway !== initialData.gateway) {
        updatedData.gateway = gateway;
		changedValues.innerHTML += `<li>Gateway: ${gateway}</li>`;
    }
	
	if (pin !== initialData.pin && pin.length === 6) { 
        updatedData.pin = pin;
        changedValues.innerHTML += `<li>PIN: ${pin}</li>`;
    }
	
	if (Object.keys(updatedData).length === 1 && updatedData.hasOwnProperty('pin')) {
    alert('Нет измененных данных для сохранения.');
    return;
	}

    if (Object.keys(updatedData).length === 0) {
        alert('Нет измененных данных для сохранения.');
        return;
    }

    if (!validateData(updatedData.ip, updatedData.mask, updatedData.gateway)) {
		invaliddata();
        return; 
    }
	
	const queryString = new URLSearchParams(updatedData).toString();

	fetch(`/save?${queryString}`, 
	{
		method: 'GET',
		headers: 
		{
			'Content-Type': 'application/json'
		}
	})
	
		setTimeout(function() 
		{
            window.location.href = "/notion.shtml"; // Перенаправление через 2 секунды
        }, 2000);
	
	
	
}



function saveDataAdmin()
{
	const macPart1 = document.getElementById('mac1').value;
    const macPart2 = document.getElementById('mac2').value;
    const macFull = `0010A180${macPart1}${macPart2}`;
    const serial = document.getElementById('serial').value;
    const updatedData = {};
    const changedValues = document.getElementById('changed-values');
    changedValues.innerHTML = ''; 

	
	if (macPart1 || macPart2) updatedData.mac = macFull;
    if (serial) updatedData.serial = serial;

	
	    if (Object.keys(updatedData).length === 0) {
        alert('Нет измененных данных для сохранения.');
        return;
    }
	
	

	
	
	if (!validateDataAdmin(macFull, serial)) {
	invaliddata();
	return; 
    }
	
	
	const queryString = new URLSearchParams(updatedData).toString();

	fetch(`/save?${queryString}`, 
	{
		method: 'GET',
		headers: 
		{
			'Content-Type': 'application/json'
		}
	})
	
		setTimeout(function() 
		{
            window.location.href = "/notion.shtml"; // Перенаправление через 2 секунды
        }, 2000);
}



function setrelay() 
{
	const queryString = "relay";
 	fetch(`/save?${queryString}`, 
	{
		method: 'GET',
		headers: 
		{
			'Content-Type': 'application/json'
		}
	})
	
			setTimeout(function() 
		{
            window.location.href = "/index.shtml"; // Перенаправление через 2 секунды
        }, 1000);

}

const mac1Element = document.getElementById('mac1');
if (mac1Element) {
  mac1Element.addEventListener('input', function() {
    if (this.value.length === 2) {
      document.getElementById('mac2').focus();
    }
  });
}

const mac2Element = document.getElementById('mac2');
if (mac2Element) {
  mac2Element.addEventListener('input', function() {
    if (this.value.length === 2) {
      document.getElementById('serial').focus();
    }
  });
}

function validateDataAdmin(macFull, serial) {
    // Проверка MAC-адреса
    if (macFull) {
        const macRegex = /^[0-9A-Fa-f]{12}$/;
        if (!macRegex.test(macFull)) {
            alert('Неверный формат MAC-адреса.');
            return false;
        }
    }

    // Проверка серийного номера: должно быть 6 цифр
    if (serial) {
        const serialRegex = /^\d{6}$/;
        if (!serialRegex.test(serial)) {
            alert('Неверный формат серийного номера.');
            return false;
        }
    }

    return true;
}

function saveChangesAdmin() 
{
	 document.getElementById('customModal').style.display = 'block';
}

function confirmChangesAdmin() {
    closeCustomModal();
    saveDataAdmin(); 
}

  function redirectToUpdatePage() {
    window.location.href = 'update.shtml'; // Перенаправление на update.shtml
  }

function editbuttoninline() {
	window.location.href = 'update.shtml';
}

function changeButtonTextBasedOnResolution() {
        var button = document.getElementById('buttonS');
        
        // Проверяем разрешение экрана
        if (window.innerWidth < 1920 && window.innerHeight < 1080) {
            button.textContent = "Обновить П.О.";
        } else {
            button.textContent = "Загрузить новую версию П.О.";
        }
    }
