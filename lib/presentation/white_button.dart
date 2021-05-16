import 'package:flutter/material.dart';

import 'aware_text.dart';

class WhiteButton extends StatelessWidget {
  final VoidCallback? onTap;
  final String? text;

  WhiteButton({@required this.onTap, @required this.text});

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
        onTap: onTap,
        child: Container(
          decoration: ShapeDecoration(
              /*shadows: [
                BoxShadow(
                    color: Colors.white60,
                    blurRadius: 15,
                    spreadRadius: 2,
                    offset: Offset(0, -3))
              ],*/
              //color: Colors.white,
              shape: RoundedRectangleBorder(
                  side: BorderSide(color: Colors.white, width: 1),
                  borderRadius: BorderRadius.circular(15))),
          child: Padding(
            padding: EdgeInsets.symmetric(vertical: 5, horizontal: 8),
            child: AwareText(
              text,
              builder: (text) => Text(
                text,
                style: TextStyle(color: Colors.white, fontSize: 12),
              ),
            ),
          ),
        ));
  }
}
