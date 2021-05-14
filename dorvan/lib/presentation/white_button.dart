import 'package:flutter/material.dart';

class WhiteButton extends StatelessWidget {
  final VoidCallback? onTap;
  final String? text;

  WhiteButton({@required this.onTap, @required this.text});

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
        onTap: onTap,
        child: Container(
          height: 30,
          width: 80,
          decoration: ShapeDecoration(
              shadows: [
                BoxShadow(
                    color: Colors.white60,
                    blurRadius: 15,
                    spreadRadius: 2,
                    offset: Offset(0, -3))
              ],
              color: Colors.white,
              shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(15))),
          child: Center(
            child: Text(
              text ?? '',
              style: TextStyle(color: Colors.black),
            ),
          ),
        ));
  }
}
