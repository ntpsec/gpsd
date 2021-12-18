// JavaScript to generate a compact date representation

//
// format date as dd-mmm-yyyyy
// example: 12-Jan-1999
//
function dateDdMmmYyyy(date)
{
  var d = date.getDate();
  var m = date.getMonth() + 1;
  var y = date.getFullYear();

  // could use splitString() here
  // but the following method is
  // more compatible
  var mmm =
    ( 1===m)?"Jan":( 2===m)?"Feb":(3===m)?"Mar":
    ( 4===m)?"Apr":( 5===m)?"May":(6===m)?"Jun":
    ( 7===m)?"Jul":( 8===m)?"Aug":(9===m)?"Sep":
    (10===m)?"Oct":(11===m)?"Nov":"Dec";

  return "" +
    (d<10?"0"+d:d) + " " + mmm + " " + y;
}


//
// get last modified date of the
// current document.
//
function dateLastModified()
{
  var lmd = document.lastModified;
  var s   = "Unknown";
  var d1;

  // check if we have a valid date
  // before proceeding
  d1 = Date.parse(lmd)
  if(0 !== d1) {
    s = "" + dateDdMmmYyyy(new Date(d1));
  }

  return s;
}

//
// finally display the last modified date
// as DD-MMM-YYYY
//
document.writeln(
  "Last modified on: " + dateLastModified() );

// End
