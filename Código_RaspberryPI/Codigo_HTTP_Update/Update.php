<?php
$target_dir = "home/pi/Desktop/Foto";

$target_file = "imagenAprocesar.jpg"
$uploadOk = 1;
$imageFileType = strtolower(pathinfo($target_file,PATHINFO_EXTENSION));

// Check if image file is a actual image or fake image
if(isset($_POST["submit"])) {
  $check = getimagesize($_FILES["imageFile"]["tmp_name"]);
  if($check !== false) {
    echo "File is an image - " . $check["mime"] . ".";
    $uploadOk = 1;
  }
  else {
    echo "File is not an image.";
    $uploadOk = 0;
  }
}

//Se comprueba si el fichero ya existe
if (file_exists($target_file)) {
	echo "Sorry, file already exists.";
	$uploadOk = 0;
}

//Se comprueba si el tamaño de la imagen es el adecuado
if ($_FILES["imageFile"]["size"] > 500000) {
  echo "El archivo es demasiado grande";
  $uploadOk = 0;
}

//Se comprueba que el formato de la imagen enviada sea el adecuado
if($imageFileType != "jpg" && $imageFileType != "png" && $imageFileType != "jpeg"
&& $imageFileType != "gif" ) {
  echo "Solo los formatos JPG, JPEG, PNG & GIF son permitido.";
  $uploadOk = 0;
}

//Si ha ocurrido algun tipo de error
if ($uploadOk == 0) {
  echo "Ha ocurrido algun tipo de error";
// Si todo ha ido bien se procede a guardar la imagen
}
else {
  //Si se ha podido guardar la imagen
  if (move_uploaded_file($_FILES["imageFile"]["tmp_name"], $target_file)) {
		//devolvemos el resultado de la ejecucion  
		echo "2";
  }//Si no se ha podido guardar la imagen
  else {
    echo "No se ha podido subir el archivo";
  }
}
?>